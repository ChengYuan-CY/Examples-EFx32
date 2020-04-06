/***************************************************************************//**
 * @file
 * @brief Capacitive touch example for BRD4181A
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"

#include "capsense.h"
#include "retargetserial.h"

static volatile uint32_t msTicks; /* counts 1ms timeTicks */

/***************************************************************************//**
 * @brief
 *   SysTick_Handler. Interrupt Service Routine for system tick counter
 ******************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;
}

/***************************************************************************//**
 * @brief
 *   Delays number of msTick Systicks (typically 1 ms)
 *
 * @param dlyTicks
 *   Number of ticks (ms) to delay
 ******************************************************************************/
static void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}

/***************************************************************************//**
 * @brief
 *   Main function
 ******************************************************************************/
int main(void)
{
  int i = 0;

  // Chip errata
  CHIP_Init();

  /* Setup SysTick timer for 1 msec interrupts  */
  if (SysTick_Config(SystemCoreClockGet() / 1000)) {
    while (1) ;
  }

  RETARGET_SerialInit();
  printf("xG21 capsense demo \r\n");
  printf("Connect a capsense pad to the PC0, press it with your finger...\r\n");

  /* Start capacitive sense buttons */
  CAPSENSE_Init();

  while (1) {
    Delay(100);
    CAPSENSE_Sense();

    if (CAPSENSE_getPressed(BUTTON0_CHANNEL)) {
      i++;
      printf("\r  %3d", i);
    }
  }
}

