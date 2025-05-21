/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "rng.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "bacnet/basic/object/device.h"
#include "bacnet/basic/sys/mstimer.h"
#include "bacnet/basic/sys/fifo.h"
#include "bacnet/basic/sys/ringbuf.h"
#include "bacnet/datalink/datalink.h"
#include "bacnet/datalink/dlmstp.h"
#include "bacnet/datalink/mstp.h"
#include "rs485.h"
#include "led.h"
#include "bacnet.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define MAX_MPDU 384

// Limit BACnet objects/devices
#define MAX_BACNET_DEVICES 5      // Default might be 128+
#define MAX_BACNET_OBJECTS 20     // Reduce if unused

#define BACDL_MSTP	1

// Disable unused services/features
#define BACNET_TIME_MASTER 0
#define BACNET_BINARY_INPUTS 0
#define BACNET_BINARY_OUTPUTS 0   // If you don’t need Binary Outputs
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t Object_Instance_Number = 10;
struct mstimer Blink_Timer;
#define FIFO_SIZE DLMSTP_MPDU_MAX
uint8_t buffer[FIFO_SIZE];
char *BACnet_Version = "1.0";
FIFO_BUFFER fifo;
/* MS/TP port */
/* MS/TP port */
static struct mstp_port_struct_t MSTP_Port;
static struct dlmstp_rs485_driver RS485_Driver = {
    .init = rs485_init,
    .send = rs485_bytes_send,
    .read = rs485_byte_available,
    .transmitting = rs485_rts_enabled,
    .baud_rate = rs485_baud_rate,
    .baud_rate_set = rs485_baud_rate_set,
    .silence_milliseconds = rs485_silence_milliseconds,
    .silence_reset = rs485_silence_reset
};
static struct dlmstp_user_data_t MSTP_User_Data;
static uint8_t Input_Buffer[DLMSTP_MPDU_MAX];
static uint8_t Output_Buffer[DLMSTP_MPDU_MAX];

int __io_putchar(int ch)
{
    (void)ch;
    return 0;
}

static void mstp_configure(void)
{
    /* initialize MSTP datalink layer */
    MSTP_Port.Nmax_info_frames = DLMSTP_MAX_INFO_FRAMES;
    MSTP_Port.Nmax_master = DLMSTP_MAX_MASTER;
    MSTP_Port.InputBuffer = Input_Buffer;
    MSTP_Port.InputBufferSize = sizeof(Input_Buffer);
    MSTP_Port.OutputBuffer = Output_Buffer;
    MSTP_Port.OutputBufferSize = sizeof(Output_Buffer);
    /* user data */
    MSTP_Port.ZeroConfigEnabled = true;
    MSTP_Port.SlaveNodeEnabled = false;
    MSTP_Port.CheckAutoBaud = false;
    MSTP_Zero_Config_UUID_Init(&MSTP_Port);
    MSTP_User_Data.RS485_Driver = &RS485_Driver;
    MSTP_Port.UserData = &MSTP_User_Data;
    dlmstp_init((char *)&MSTP_Port);
    if (MSTP_Port.ZeroConfigEnabled) {
        dlmstp_set_mac_address(127);
    } else {
        /* FIXME: get the address from hardware DIP or from EEPROM */
        dlmstp_set_mac_address(1);
    }
    /* FIXME: get the baud rate from hardware DIP or from EEPROM */
    dlmstp_set_baud_rate(DLMSTP_BAUD_RATE_DEFAULT);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  mstimer_init();
  led_init();
  Device_Set_Object_Instance_Number(103);
  mstimer_set(&Blink_Timer, 500);



  srand(Device_Object_Instance_Number());
  Device_UUID_Init();
  Device_UUID_Get(MSTP_Port.UUID, sizeof(MSTP_Port.UUID));
  /* initialize MSTP datalink layer */
  MSTP_Port.Nmax_info_frames = DLMSTP_MAX_INFO_FRAMES;
  MSTP_Port.Nmax_master = DLMSTP_MAX_MASTER;
  MSTP_Port.InputBuffer = Input_Buffer;
  MSTP_Port.InputBufferSize = sizeof(Input_Buffer);
  MSTP_Port.OutputBuffer = Output_Buffer;
  MSTP_Port.OutputBufferSize = sizeof(Output_Buffer);
  /* choose from non-volatile configuration for zero-config or slave mode */
  MSTP_Port.ZeroConfigEnabled = true;
  MSTP_Port.Zero_Config_Preferred_Station = 0;
  MSTP_Port.SlaveNodeEnabled = false;
  MSTP_Port.CheckAutoBaud = true;
  /* user data */
  MSTP_User_Data.RS485_Driver = &RS485_Driver;
  MSTP_Port.UserData = &MSTP_User_Data;
//  for(;;){
//	  printf("Test Bacnet\r\n");
//	  HAL_Delay(500);
//  }
  rs485_init();
  mstp_configure();
  dlmstp_init((char *)&MSTP_Port);
  if (MSTP_Port.ZeroConfigEnabled) {
      /* set node to monitor address */
      dlmstp_set_mac_address(103);
  } else {
      /* FIXME: get the address from hardware DIP or from EEPROM */
      dlmstp_set_mac_address(1);
  }
  if (!MSTP_Port.CheckAutoBaud) {
      /* FIXME: get the baud rate from hardware DIP or from EEPROM */
      dlmstp_set_baud_rate(DLMSTP_BAUD_RATE_DEFAULT);
  }
  /* initialize application layer*/
 bacnet_init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      if (mstimer_expired(&Blink_Timer)) {
          mstimer_reset(&Blink_Timer);
          led_toggle(LED_LD1);
           led_toggle(LED_LD2);

      }
      led_task();
      bacnet_task();
      HAL_UART_Receive_DMA(&huart2, &UartDMABuffer, 1);


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
