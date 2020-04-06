/***************************************************************************//**
 * @file
 * @brief capsense configuration parameters.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#ifndef __SILICON_LABS_CAPSENSCONFIG_H__
#define __SILICON_LABS_CAPSENSCONFIG_H__
#ifdef __cplusplus
extern "C" {
#endif

/* Use ACMP0 module for capsense */
#define ACMP_CAPSENSE                           ACMP0
#define ACMP_CAPSENSE_CMUCLOCK                  cmuClock_ACMP0

#define CAPSENSE_CHANNELS       { acmpInputPC0 }
#define BUTTON0_CHANNEL         0             /**< Button 0 channel */
#define ACMP_CHANNELS           1             /**< Number of channels in use for capsense */
#define NUM_SLIDER_CHANNELS     0             /**< The kit does not have a slider */

#define ACMP_CAPSENSE_INPUTPORT                 gpioPortC
#define ACMP_CAPSENSE_INPUTPIN                  0

#define PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE     PRS_ASYNC_CH_CTRL_SOURCESEL_ACMP0
#define PRS_CH_CTRL_SIGSEL_ACMPOUT_CAPSENSE     PRS_ASYNC_CH_CTRL_SIGSEL_ACMP0OUT

/* PRS Channel, the ACMP output is an asynchronous PRS producer, and route to TIMER1 CC1 */
/* Pls consider the available PRS channel if need to output the ACMP output to specified gpio pins for debugging purpose*/
#define PRS_CH_ACMP_TO_TIMER                    6

#define SAMPLE_TIME                             1  /* Sampling time for each capacitive sense button, unit is ms*/
/* On the SLSTK3401A the touch buttons are connected to PB11 and PB12.
 *
 * Pin  | APORT Channel (for ACMP0)
 * -------------------------
 * PB11 | APORT4XCH27
 * PB12 | APORT3XCH28
 *
 */


#ifdef __cplusplus
}
#endif
#endif /* __SILICON_LABS_CAPSENSCONFIG_H__ */
