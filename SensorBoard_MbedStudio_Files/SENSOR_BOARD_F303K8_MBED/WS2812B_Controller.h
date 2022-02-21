/*
 *
 ******************************************************************************
 *
 *  WS2812 Control function (C)Shun Hattori, 2021
 *	@brief generate protocol pulses to the module connected to PA_10
 *
 ******************************************************************************
 *
 */
#include <cstdint>
#include <mbed.h>

#ifndef WS2812CF_DEF
#define WS2812CF_DEF

void WS2812_apply_color(register uint32_t byte);
void WS2812_write_PA10(register uint8_t byte);

inline void WS2812_apply_color(register uint32_t byte) {
  //	for (int i = 3; i > 0; i--) {
  //		WS2812_write_PA10((byte >> ((i - 1) * 8)) & 0xFF);
  //	}
  // G7 G6 G5 G4 G3 G2 G1 G0 R7 R6 R5 R4 R3 R2 R1 R0 B7 B6 B5 B4 B3 B2 B1 B0
  __disable_irq();
  WS2812_write_PA10((byte >> 8) & 0xFF);  // RED 8bit
  WS2812_write_PA10((byte >> 16) & 0xFF); // GREEN 8bit
  WS2812_write_PA10((byte >> 0) & 0xFF);  // BLUE 8bit
  __enable_irq();
}

inline void WS2812_write_PA10(register uint8_t byte) {
  for (int i = 0; i < 8; i++) {
    // GPIOA->BRR = (uint32_t)(GPIO_PIN_10); // GPIOA, GPIO_PIN_5 = PA_5 enable
    GPIOA->BSRR = (uint32_t)GPIO_PIN_10;
    if (byte & 0x80) {
      //   for (int j = 0; j < 14; j++) // 800 +- 150
      for (int j = 0; j < 7; j++)
        __NOP();
      GPIOA->BRR = (uint32_t)(GPIO_PIN_10);
      //   for (int j = 0; j < 7; j++) // 450 +- 150
      for (int j = 0; j < 2; j++)
        __NOP();
    } else {
      //   for (int j = 0; j < 6; j++) // 400 +- 150
      for (int j = 0; j < 2; j++)
        __NOP();
      GPIOA->BRR = (uint32_t)(GPIO_PIN_10);
      //   for (int j = 0; j < 15; j++) // 850 +- 150
      for (int j = 0; j < 7; j++)
        __NOP();
    }
    byte = byte << 1;
  }
}

#endif