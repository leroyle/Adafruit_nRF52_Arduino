#ifndef _VARIANT_SEEED_XIAO_NRF52840_SENSE
#define _VARIANT_SEEED_XIAO_NRF52840_SENSE

#define SEEED_XIAO_NRF52840
#define ANALOG_CONFIG

/*----------------------------------------------------------------------------
 *        Headers
 ----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

  /* Analog reference options
   * Different possibilities available combining Reference and Gain
   */
  /* from MBED
  enum _AnalogReferenceMode
  {
    AR_VDD,         // 3.3 V
    AR_INTERNAL,    // 0.6 V
    AR_INTERNAL1V2, // 1.2 V
    AR_INTERNAL2V4  // 2.4 V
  };
  */

  /* Analog acquisition time options */
  enum _AnalogAcquisitionTime
  {
    AT_3_US,
    AT_5_US,
    AT_10_US, // Default value
    AT_15_US,
    AT_20_US,
    AT_40_US
  };

// Frequency of the board main oscillator
#define VARIANT_MAINOSC (32768ul)

// Master clock frequency
#define VARIANT_MCK (64000000ul)

// Pins
// ----

// Number of pins defined in PinDescription array
#ifdef __cplusplus
  extern "C" unsigned int PINCOUNT_fn();
#endif
#define PINS_COUNT (33) //  (PINCOUNT_fn())
#define NUM_DIGITAL_PINS (33)
#define NUM_ANALOG_INPUTS (7u)
#define NUM_ANALOG_OUTPUTS (0u)

// extern PinName digitalPinToPinName(pin_size_t P);

// LEDs
// ----
#define PIN_LED (11u)
#define LED_BUILTIN PIN_LED
#define LEDR (11u)
#define LEDG (12u)
#define LEDB (13u)
#define LED_PWR (11u)

#define LED_RED LEDR
#define LED_GREEN LEDG
#define LED_BLUE LEDB
#define LED_POWER LED_PWR

#define PIN_LED1 PIN_LED
#define PIN_LED2 LEDG
#define LED_STATE_ON 0  // State when LED is on
#define LED_STATE_OFF 1 // State when LED is off

// Analog pins
// -----------
#define PIN_A0 (0u) // P0_2
#define PIN_A1 (1u) // P0_3
#define PIN_A2 (2u) // P0_28
#define PIN_A3 (3u) // P0_29
#define PIN_A4 (4u) // P0_4/SDA
#define PIN_A5 (5u) // P0_5/SCL
#define PIN_VBAT (32u)
// #define PIN_A6 (31)  // P0_31/SCL

  static const uint8_t A0 = PIN_A0;
  static const uint8_t A1 = PIN_A1;
  static const uint8_t A2 = PIN_A2;
  static const uint8_t A3 = PIN_A3;
  static const uint8_t A4 = PIN_A4;
  static const uint8_t A5 = PIN_A5;

#define ADC_RESOLUTION 12

// defines from /home/leroy/.platformio/packages/framework-arduino-mbed/variants/SEEED_XIAO_NRF52840_SENSE/variant.cpp

// Digital pins
// -----------
#define D0 (0u)
#define D1 (1u)
#define D2 (2u)
#define D3 (3u)
#define D4 (4u)
#define D5 (5u)
#define D6 (6u)
#define D7 (7u)
#define D8 (8u)
#define D9 (9u)
#define D10 (10u)
#define D29 (29u)
#define D30 (30u)

// Non user pins
#define D11 (26u) // P0_26/LED RED
#define D12 (30u) // P0_30/LED GREEN
#define D13 (6u)  // P0_6/LED  BLUE

// XIAO BLE Sense only
// LSM6DS3TR   IMU
// #define D14 (8u)  // P1_8/6D_PWR
// #define D15 (27u) // P0_27/6D_I2C_SCL
// #define D16 (7u)  // P0_7/6D_I2C_SDA
// #define D17 (11u) // P0_11/6D_INT1
// PDM  Microphone
// #define D18 (42u) // P1_10/PDM PWR
// #define D19 (33u) // P1_0/CLK
// #define D20 (16u) // P0_16/PDM DIN
//
// Battery charger
// #define D21 (13u) // P0_13/HICHG
// #define D22 (17u) // P0_17/~CHG
//
// QSPI  on board flash
// #define D23 (21u) // P0_21/QSPI_SCK
// #define D24 (25u) // P0_25/QSPI_CSN
// #define D25 (20u) // P0_20/QSPI_SIO_0 DI
// #define D26 (24u) // P0_24/QSPI_SIO_1 D0
// #define D27 (22u) // P0_22/QSPI_SIO_2 WP
// #define D28 (23u) // P0_23/QSPI_SIO_3 HOLD

// NFC
// #define D29 (9u)  // P0_9/I2C_PULL
// #define D30 (10u) // P0_10/VDD_ENV_ENABLE

// VBAT
// #define D31 (14u) // P0_14/VBAT_ENABLE
// #define D32 (31u) // P0_31/VBAT_READ

/*
 * Serial interfaces^M
 */
// Serial (EDBG)
#define PIN_SERIAL_RX (7ul)
#define PIN_SERIAL_TX (6ul)

/*
 * Serial interfaces
 */
#define PIN_SERIAL1_RX (43l)
#define PIN_SERIAL1_TX (44l)

// Needed for SD library
#define SDCARD_SPI SPI
#define SDCARD_SS_PIN PIN_SPI_SS

// SPI
#define PIN_SPI_MISO (9u)
#define PIN_SPI_MOSI (10u)
#define PIN_SPI_SCK (8u)
#define PIN_SPI_SS (2u)

static const uint8_t SS = PIN_SPI_SS; // SPI Slave SS not used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK = PIN_SPI_SCK;

// Wire

#define PIN_WIRE_SDA (4u)
#define PIN_WIRE_SCL (5u)

#define PIN_WIRE_SDA1 (16u)
#define PIN_WIRE_SCL1 (15u)

#define PIN_LSM6DS3TR_C_POWER (14u)
#define PIN_LSM6DS3TR_C_INT1 (17u)

// PDM Interfaces
// ---------------
#define PIN_PDM_PWR (18u)
#define PIN_PDM_CLK (19u)
#define PIN_PDM_DIN (20u)

#define USE_LFXO // Board uses 32khz crystal for LF^M

// QSPI Pins
#define PIN_QSPI_SCK (23u)
#define PIN_QSPI_CS (24u)
#define PIN_QSPI_IO0 (25u)
#define PIN_QSPI_IO1 (26u)
#define PIN_QSPI_IO2 (27u)
#define PIN_QSPI_IO3 (28u)

// On-board QSPI Flash
#define EXTERNAL_FLASH_DEVICES P25Q16H
#define EXTERNAL_FLASH_USE_QSPI

// From MBED define
// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.^

// Mbed specific defines
#define SERIAL_HOWMANY 1
#define SERIAL1_TX (digitalPinToPinName(PIN_SERIAL_TX))
#define SERIAL1_RX (digitalPinToPinName(PIN_SERIAL_RX))

#define SERIAL_CDC 1
#define HAS_UNIQUE_ISERIAL_DESCRIPTOR
#define BOARD_VENDORID 0x2886
#define BOARD_PRODUCTID 0x8045
#define BOARD_NAME "Seeed XIAO nRF52840 Sense"

// #define DFU_MAGIC_SERIAL_ONLY_RESET 0x4E // 0xb0

#define WIRE_HOWMANY 2

#define I2C_SDA                         (digitalPinToPinName(PIN_WIRE_SDA))
#define I2C_SCL                         (digitalPinToPinName(PIN_WIRE_SCL))
#define I2C_SDA1                        (digitalPinToPinName(PIN_WIRE_SDA1))
#define I2C_SCL1                        (digitalPinToPinName(PIN_WIRE_SCL1))

#define SPI_HOWMANY                     1

#define SPI_MISO                        (digitalPinToPinName(PIN_SPI_MISO))
#define SPI_MOSI                        (digitalPinToPinName(PIN_SPI_MOSI))
#define SPI_SCK                         (digitalPinToPinName(PIN_SPI_SCK))

// #define digitalPinToPort(P)             (digitalPinToPinName(P)/32)

uint8_t getUniqueSerialNumber(uint8_t* name);
void _ontouch1200bps_();

//#define PIN_ENABLE_I2C_PULLUP      (32u)
//#define PIN_ENABLE_SENSORS_3V3     (33u)
//
//#define PIN_INT_APDS (26u)

// PDM Interfaces
// ---------------
#define PIN_PDM_PWR (18u)
#define PIN_PDM_CLK (19u)
#define PIN_PDM_DIN (20u)

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif // _VARIANT_SEEED_XIAO_NRF52840_SENSE
