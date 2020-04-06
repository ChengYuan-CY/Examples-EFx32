/***************************************************************************//**
 * @file
 * @brief Capacitive sense driver
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

#include "em_device.h"
#include "em_acmp.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_prs.h"
#include "em_timer.h"
#include "capsense.h"

/***************************************************************************//**
 * @addtogroup kitdrv
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup CapSense
 * @brief Capacitive sensing driver
 *
 * @details
 *  Capacitive sensing driver using TIMER and ACMP peripherals.
 *
 * @{
 ******************************************************************************/

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

/** The current channel we are sensing. */
static volatile uint8_t currentChannel;
/** Flag for measurement completion. */
static volatile bool measurementComplete;

#if defined(CAPSENSE_CH_IN_USE)
/**************************************************************************//**
 * @brief
 *   A bit vector which represents the channels to iterate through
 *
 * @note
 *   This API is deprecated and new application should define
 *   CAPSENSE_CHANNELS instead of CAPSENSE_CH_IN_USE.
 *
 * @param ACMP_CHANNELS
 *   Vector of channels.
 *****************************************************************************/
static const bool channelsInUse[ACMP_CHANNELS] = CAPSENSE_CH_IN_USE;
#elif defined(CAPSENSE_CHANNELS)
/**************************************************************************//**
 * @brief
 *   An array of channels that the capsense driver should iterate through.
 *
 * @param CAPSENSE_CHANNELS
 *   Initializer list that contains all the channels that the application
 *   would like to use for capsense.
 *
 * @param ACMP_CHANNELS
 *   The number of channels in the array
 *****************************************************************************/
static const ACMP_Channel_TypeDef channelList[ACMP_CHANNELS] = CAPSENSE_CHANNELS;
#endif

/**************************************************************************//**
 * @brief The NUM_SLIDER_CHANNELS specifies how many of the ACMP_CHANNELS
 *        are used for a touch slider
 *****************************************************************************/
#if !defined(NUM_SLIDER_CHANNELS)
#define NUM_SLIDER_CHANNELS 4
#endif

/**************************************************************************//**
 * @brief This vector stores the latest read values from the ACMP
 * @param ACMP_CHANNELS Vector of channels.
 *****************************************************************************/
static volatile uint32_t channelValues[ACMP_CHANNELS] = { 0 };

/**************************************************************************//**
 * @brief  This stores the maximum values seen by a channel
 * @param ACMP_CHANNELS Vector of channels.
 *****************************************************************************/
static volatile uint32_t channelMaxValues[ACMP_CHANNELS] = { 0 };

/** @endcond */

/**************************************************************************//**
 * @brief
 *   TIMER0 interrupt handler.
 *
 * @details
 *   When TIMER0 expires the number of pulses on TIMER1 is inserted into
 *   channelValues. If this values is bigger than what is recorded in
 *   channelMaxValues, channelMaxValues is updated.
 *   Finally, the next ACMP channel is selected.
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{
  uint32_t count;

  // Acknowledge the interrupt
  uint32_t flags = TIMER_IntGet(TIMER0);
  TIMER_IntClear(TIMER0, flags);

  /* Stop timers */
  TIMER_Enable(TIMER0, false);
  TIMER_Enable(TIMER1, false);

  /* Read out value of TIMER1 */
  count = TIMER_CounterGet(TIMER1);

  /* Store value in channelValues */
  channelValues[currentChannel] = count;

  /* Update channelMaxValues */
  if (count > channelMaxValues[currentChannel]) {
    channelMaxValues[currentChannel] = count;
  }

  measurementComplete = true;
}

/**************************************************************************//**
 * @brief Get the current channelValue for a channel
 * @param channel The channel.
 * @return The channelValue.
 *****************************************************************************/
uint32_t CAPSENSE_getVal(uint8_t channel)
{
  return channelValues[channel];
}

/**************************************************************************//**
 * @brief Get the current normalized channelValue for a channel
 * @param channel The channel.
 * @return The channel value in range (0-256).
 *****************************************************************************/
uint32_t CAPSENSE_getNormalizedVal(uint8_t channel)
{
  uint32_t max = channelMaxValues[channel];
  return (channelValues[channel] << 8) / max;
}

/**************************************************************************//**
 * @brief Get the state of the Gecko Button
 * @param channel The channel.
 * @return true if the button is "pressed"
 *         false otherwise.
 *****************************************************************************/
bool CAPSENSE_getPressed(uint8_t channel)
{
  uint32_t treshold;
  /* Treshold is set to 12.5% below the maximum value */
  /* This calculation is performed in two steps because channelMaxValues is
   * volatile. */
  treshold  = channelMaxValues[channel];
  treshold -= channelMaxValues[channel] >> 2;

  if (channelValues[channel] < treshold) {
    return true;
  }
  return false;
}

/**************************************************************************//**
 * @brief Get the position of the slider
 * @return The position of the slider if it can be determined,
 *         -1 otherwise.
 *****************************************************************************/
int32_t CAPSENSE_getSliderPosition(void)
{
  int      i;
  int      minPos = -1;
  uint32_t minVal = 224; /* 0.875 * 256 */
  /* Values used for interpolation. There is two more which represents the edges.
   * This makes the interpolation code a bit cleaner as we do not have to make special
   * cases for handling them */
  uint32_t interpol[(NUM_SLIDER_CHANNELS + 2)];
  for (i = 0; i < (NUM_SLIDER_CHANNELS + 2); i++) {
    interpol[i] = 255;
  }

  /* The calculated slider position. */
  int position;

  /* Iterate through the slider bars and calculate the current value divided by
   * the maximum value multiplied by 256.
   * Note that there is an offset of 1 between channelValues and interpol.
   * This is done to make interpolation easier.
   */
  for (i = 1; i < (NUM_SLIDER_CHANNELS + 1); i++) {
    /* interpol[i] will be in the range 0-256 depending on channelMax */
    interpol[i]  = channelValues[i - 1] << 8;
    interpol[i] /= channelMaxValues[i - 1];
    /* Find the minimum value and position */
    if (interpol[i] < minVal) {
      minVal = interpol[i];
      minPos = i;
    }
  }
  /* Check if the slider has not been touched */
  if (minPos == -1) {
    return -1;
  }

  /* Start position. Shift by 4 to get additional resolution. */
  /* Because of the interpol trick earlier we have to substract one to offset that effect */
  position = (minPos - 1) << 4;

  /* Interpolate with pad to the left */
  position -= ((256 - interpol[minPos - 1]) << 3)
              / (256 - interpol[minPos]);

  /* Interpolate with pad to the right */
  position += ((256 - interpol[minPos + 1]) << 3)
              / (256 - interpol[minPos]);

  return position;
}

/**************************************************************************//**
 * @brief
 *   Start a capsense measurement of a specific channel and waits for
 *   it to complete.
 *****************************************************************************/
static void CAPSENSE_Measure(ACMP_Channel_TypeDef channel)
{
  /* Set up this channel in the ACMP. */
  ACMP_CapsenseChannelSet(ACMP_CAPSENSE, channel);

  /* Reset timers */
  TIMER_CounterSet(TIMER0, 0);
  TIMER_CounterSet(TIMER1, 0);

  measurementComplete = false;

  /* Start timers */
  TIMER_Enable(TIMER0, true);
  TIMER_Enable(TIMER1, true);

  /* Wait for measurement to complete */
  while ( measurementComplete == false ) {
    EMU_EnterEM1();
  }
}

/**************************************************************************//**
 * @brief
 *   This function iterates through all the capsensors and reads and
 *   initiates a reading. Uses EM1 while waiting for the result from
 *   each sensor.
 *****************************************************************************/
void CAPSENSE_Sense(void)
{
  /* Use the default STK capacative sensing setup and enable it */
  ACMP_Enable(ACMP_CAPSENSE);

#if defined(CAPSENSE_CHANNELS)
  /* Iterate through only the channels in the channelList */
  for (currentChannel = 0; currentChannel < ACMP_CHANNELS; currentChannel++) {
    CAPSENSE_Measure(channelList[currentChannel]);
  }
#else
  /* Iterate through all channels and check which channel is in use */
  for (currentChannel = 0; currentChannel < ACMP_CHANNELS; currentChannel++) {
    /* If this channel is not in use, skip to the next one */
    if (!channelsInUse[currentChannel]) {
      continue;
    }

    CAPSENSE_Measure((ACMP_Channel_TypeDef) currentChannel);
  }
#endif

  /* Disable ACMP while not sensing to reduce power consumption */
  ACMP_Disable(ACMP_CAPSENSE);
}


/**************************************************************************//**
 * @brief GPIO initialization
 *****************************************************************************/
void initGPIO(void)
{
  // Configure ACMP input pins
  GPIO_PinModeSet(ACMP_CAPSENSE_INPUTPORT, ACMP_CAPSENSE_INPUTPIN, gpioModeInput, 1);

  // Debug usage
  // Output the ACMP result to PC1
  GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 0);

}

/**************************************************************************//**
 * @brief TIMER initialization
 *****************************************************************************/
void initTIMER0(void)
{
  uint32_t timerFreq = 0;
  // Initialize the timer
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  // Configure TIMER0 Compare/Capture for output compare
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;

  timerInit.prescale = timerPrescale512;
  timerInit.enable = false;
  timerInit.oneShot = true;

  timerCCInit.mode = timerCCModeOff;
  timerCCInit.cmoa = timerOutputActionNone;

  // configure, but do not start timer
  TIMER_Init(TIMER0, &timerInit);
  TIMER_InitCC(TIMER0, 0, &timerCCInit);

  // Set Top value
  // Note each overflow event constitutes 1/2 the signal period
  timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0)/(timerInit.prescale + 1);
  int topValue = (timerFreq * SAMPLE_TIME)/1000;
  TIMER_TopSet (TIMER0, topValue);

  /* Enable TIMER0 overflow interrupt */
  /* Will not start TIMER until start capsense capture*/
  TIMER_IntEnable(TIMER0, TIMER_IEN_OF);
  NVIC_EnableIRQ(TIMER0_IRQn);
}

/**************************************************************************//**
 * @brief TIMER initialization
 *****************************************************************************/
void initTIMER1(void)
{
  // Initialize timer with defined prescale
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  // Configure TIMER0 Compare/Capture settings
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;

  timerInit.enable = false;
  timerInit.prescale = timerPrescale1024;
  timerInit.clkSel = timerClkSelCC1;

  // Set event on every edge
  timerCCInit.eventCtrl = timerEventRising;
  timerCCInit.edge = timerEdgeBoth;
  timerCCInit.mode = timerCCModeCapture;
  timerCCInit.prsInput = true;
  timerCCInit.prsSel = PRS_CH_ACMP_TO_TIMER;
  timerCCInit.prsInputType = timerPrsInputAsyncPulse;

  TIMER_Init(TIMER1, &timerInit);

  TIMER_InitCC(TIMER1, 1, &timerCCInit);

  /* Will not start TIMER until start capsense capture*/
//  TIMER_Enable(TIMER1, true);
}

/**************************************************************************//**
 * @brief
 *   Initializes the capacitive sense system.
 *
 * @details
 *   Capacitive sensing uses two timers: TIMER0 and TIMER1 as well as ACMP.
 *   ACMP is set up in cap-sense (oscillator mode).
 *   TIMER1 counts the number of pulses generated by ACMP_CAPSENSE.
 *   When TIMER0 expires it generates an interrupt.
 *   The number of pulses counted by TIMER1 is then stored in channelValues
 *****************************************************************************/
void CAPSENSE_Init(void)
{
  /* Use the default STK capacative sensing setup */
  ACMP_CapsenseInit_TypeDef capsenseInit = ACMP_CAPSENSE_INIT_DEFAULT;

  /* Enable GPIO, TIMER0, TIMER1, ACMP_CAPSENSE and PRS clock */
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);
  CMU_ClockEnable(cmuClock_TIMER1, true);
#if defined(ACMP_CAPSENSE_CMUCLOCK)
  CMU_ClockEnable(ACMP_CAPSENSE_CMUCLOCK, true);
#endif
  CMU_ClockEnable(cmuClock_PRS, true);

  /* Initialize GPIO mode*/
  initGPIO();

  /* Set up ACMP1 in capsense mode */
  capsenseInit.enable = false;
  ACMP_CapsenseInit(ACMP_CAPSENSE, &capsenseInit);

  // ACMP0 input pin is PC0, so allocate Analog Bus (ABUS) CDEVEN0 to ACMP0 to be able to use the input from PC0
  // e.g., if ACMP0 input pin is PA0, should set the GPIO->ABUSALLOC = GPIO_ABUSALLOC_AEVENx_ACMP0 (the "x" can be 0 or 1)
  // e.g., if ACMP0 input pin is PD3, should set the GPIO->CDBUSALLOC = GPIO_CDBUSALLOC_CDODDx_ACMP0 (the "x" can be 0 or 1)
  GPIO->CDBUSALLOC = GPIO_CDBUSALLOC_CDEVEN0_ACMP0;

#ifdef DEBUG_ACMP_OUTPUT
  // Don't need to enable ACMP and set channel here, will set it while starting capsense measure
  // If you'd like to output the ACMP to gpios, enable it here.
  ACMP_Enable(ACMP_CAPSENSE);
  ACMP_CapsenseChannelSet(ACMP_CAPSENSE, ACMP_CAPSENSE_CHANNEL);

  // To be able to probe the output we can send the ACMP output to a pin directly.
  ACMP_GPIOSetup(ACMP_CAPSENSE, gpioPortC, 1, true, false);
#endif

  /*Set up PRS channel x to trigger on ACMP0 output*/
  PRS_SourceAsyncSignalSet(
      PRS_CH_ACMP_TO_TIMER,
      PRS_ASYNC_CH_CTRL_SOURCESEL_ACMP0,
      PRS_ASYNC_CH_CTRL_SIGSEL_ACMP0OUT);

  // Do not apply any logic on the PRS Channel
  PRS_Combine(PRS_CH_ACMP_TO_TIMER, PRS_CH_ACMP_TO_TIMER, prsLogic_A);

//#ifdef DEBUG_PRS_OUTPUT
#if 1
    // To be able to probe the output we can send the PRS output to a pin directly.
  // It's another way to output the ACMP.
  // Note, not all of the gpio pins are available for the specified PRS channel, please check the datasheet
  // e.g., PRS.ASYNCH0 - PRS.ASYNCH5 cannot only output to PA/PB port
    PRS_PinOutput(PRS_CH_ACMP_TO_TIMER, prsTypeAsync, gpioPortC , 1);
#endif

  // Select PRS channel for Timer1 CC1
  PRS_ConnectConsumer(PRS_CH_ACMP_TO_TIMER, prsTypeAsync, prsConsumerTIMER1_CC1);

  /* Initialize TIMER0 */
  initTIMER0();

  /* Initialize TIMER1 as counter */
  initTIMER1();
}

/** @} (end group CapSense) */
/** @} (end group kitdrv) */
