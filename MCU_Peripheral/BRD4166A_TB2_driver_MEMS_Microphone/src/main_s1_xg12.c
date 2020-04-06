/**************************************************************************//**
 * @main.c
 * @brief This project demonstrates DMA-driven use of the USART in
 * synchronous (SPI) The main loop starts the LDMA channels, which transmit
 * the specified number of bytes and receive the byte that is shifted in
 * with each outgoing one.
 *
 * The pins used in this example are defined below and are described in
 * the accompanying readme.txt file.
 *
 * @version 0.0.1
 ******************************************************************************
 * @section License
 * <b>Copyright 2018 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_ldma.h"
#include "em_usart.h"

#include "mic.h"

//#define MIC_SAMPLE_RATE            1000
//#define MIC_SAMPLE_BUFFER_SIZE     512

#define MIC_SAMPLE_RATE            8000
#define MIC_SAMPLE_BUFFER_SIZE     2048

static uint16_t micSampleBuffer[MIC_SAMPLE_BUFFER_SIZE];

/**************************************************************************//**
 * @brief
 *    Main function
 *****************************************************************************/
int main(void)
{
  // Chip errata
  CHIP_Init();

  // Initialize microphone
  MIC_init(MIC_SAMPLE_RATE, micSampleBuffer, MIC_SAMPLE_BUFFER_SIZE);

  while (1)
  {

  }

}
