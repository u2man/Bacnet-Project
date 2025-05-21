/**
 * @file
 * @brief BACnet stack initialization and task processing
 * @author Steve Karg
 * @date 2021
 * @copyright SPDX-License-Identifier: MIT
 */
#ifndef BACNET_H
#define BACNET_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// Disable BACnet/IP
#define BACNET_BIP 0
#define BACNET_BIP6 0
#define BACNET_BT 0

// Enable BACnet MSTP
#define BACNET_MSTP 1
#define BACDL_MSTP 1
void bacnet_init(void);
void bacnet_task(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif
