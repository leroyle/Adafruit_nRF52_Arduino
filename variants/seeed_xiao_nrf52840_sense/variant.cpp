/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  Copyright (c) 2018, Adafruit Industries (adafruit.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"
#include "wiring_constants.h"
#include "wiring_digital.h"
#include "nrf.h"


const uint32_t g_ADigitalPinMap[] = {
  // D0 - D10
  2,   //  A0  P0.02
  3,   // A1  P0.03
  28,  // A2  P0.28
  29,  // A3  P0.29
  4,   // A4/SDA  P0.04
  5,   // A5/SCL  P0.05
  43,  // TX  P1.11
  44,  // RX  P1.12
  45,  // SCK  P1.13
  46,  // MISO  P1.14
  47,  // MOSI  P1.15

  // Non User LEDS D11-D13
  26,  // LED RED  P0_26
  30,  // LED GREEN  P0_30
  6,   // LED BLUE  P0_6

  // LSM6DS3TR  IMU  D14 -D17
  8,   // 6D_PWR
  27,  // 6D_I2C_SCL
  7,   //  6D_I2C_SDA
  11,  // 6D_INT1

  // PDM  Microphone  D18-D20
  42,  // PDM PWR
  33,  // CLK
  16,  // PDM DIN

  // Battery charger  D21-D22
  13,  // HICHG
  17,  // ~CHG

  // QSPI  on board flash  D23-D28
  21,  // QSPI_SCK
  25,  // QSPI_CSN
  20,  // QSPI_SIO_0 DI
  24,  // QSPI_SIO_1 D0
  22,  // QSPI_SIO_2 WP
  23,  // QSPI_SIO_3 HOLD
 
  // NFC   D29-D30
  9,   // I2C_PULL
  10,  // VDD_ENV_ENABLE

  // VBAT  D31-D32
  14,  // VBAT_ENABLE
  31   // VBAT_READ
};


void initVariant()
{
	// LED1 & LED2
	pinMode(PIN_LED1, OUTPUT);
	ledOff(PIN_LED1);

	pinMode(PIN_LED2, OUTPUT);
	ledOff(PIN_LED2);
}

extern "C" {
  unsigned int PINCOUNT_fn() {
    return (sizeof(g_ADigitalPinMap) / sizeof(g_ADigitalPinMap[0]));
  }
}

/*
void initVariant_MBED() {
  // turn power LED on
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR, HIGH);

  // Errata Nano33BLE - I2C pullup is controlled by the SWO pin.
  // Configure the TRACEMUX to disable routing SWO signal to pin.
  NRF_CLOCK->TRACECONFIG = 0;

  // FIXME: bootloader enables interrupt on COMPARE[0], which we don't handle
  // Disable it here to avoid getting stuck when OVERFLOW irq is triggered
  nrf_rtc_event_disable(NRF_RTC1, NRF_RTC_INT_COMPARE0_MASK);
  nrf_rtc_int_disable(NRF_RTC1, NRF_RTC_INT_COMPARE0_MASK);

  // FIXME: always enable I2C pullup and power @startup
  // Change for maximum powersave
  // pinMode(PIN_ENABLE_SENSORS_3V3, OUTPUT);
  // pinMode(PIN_ENABLE_I2C_PULLUP, OUTPUT);

  // digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH);
  // digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH);

  // Disable UARTE0 which is initially enabled by the bootloader
  // nrf_uarte_task_trigger(NRF_UARTE0, NRF_UARTE_TASK_STOPRX); 
  // while (!nrf_uarte_event_check(NRF_UARTE0, NRF_UARTE_EVENT_RXTO)) ;
  NRF_UARTE0->ENABLE = 0; 
  NRF_UART0->ENABLE = 0; 
NRF_PWM_Type* PWM[] = {
    NRF_PWM0, NRF_PWM1, NRF_PWM2
#ifdef NRF_PWM3
    ,NRF_PWM3
#endif
  };

  for (unsigned int i = 0; i < (sizeof(PWM)/sizeof(PWM[0])); i++) {
    PWM[i]->ENABLE = 0;
    PWM[i]->PSEL.OUT[0] = 0xFFFFFFFFUL;
  } 
}
*/
