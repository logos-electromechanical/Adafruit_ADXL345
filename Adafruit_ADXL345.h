/**************************************************************************/
/*!
    @file     Adafruit_ADS1015.h
    @author   K. Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    This is a library for the Adafruit ADS1015 breakout board
    ----> https://www.adafruit.com/products/???

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>
#include <Wire.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ADXL345_ADDRESS                 (0x53)    // Assumes ALT address pin low
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define ADXL345_REG_DEVID               (0x00)    // Device ID
    #define ADXL345_REG_THRESH_TAP          (0x1D)    // Tap threshold
    #define ADXL345_REG_OFSX                (0x1E)    // X-axis offset
    #define ADXL345_REG_OFSY                (0x1F)    // Y-axis offset
    #define ADXL345_REG_OFSZ                (0x20)    // Z-axis offset
    #define ADXL345_REG_DUR                 (0x21)    // Tap duration
    #define ADXL345_REG_LATENT              (0x22)    // Tap latency
    #define ADXL345_REG_WINDOW              (0x23)    // Tap window
    #define ADXL345_REG_THRESH_ACT          (0x24)    // Activity threshold
    #define ADXL345_REG_THRESH_INACT        (0x25)    // Inactivity threshold
    #define ADXL345_REG_TIME_INACT          (0x26)    // Inactivity time
    #define ADXL345_REG_ACT_INACT_CTL       (0x27)    // Axis enable control for activity and inactivity detection
    #define ADXL345_REG_THRESH_FF           (0x28)    // Free-fall threshold
    #define ADXL345_REG_TIME_FF             (0x29)    // Free-fall time
    #define ADXL345_REG_TAP_AXES            (0x2A)    // Axis control for single/double tap
    #define ADXL345_REG_ACT_TAP_STATUS      (0x2B)    // Source for single/double tap
    #define ADXL345_REG_BW_RATE             (0x2C)    // Data rate and power mode control
    #define ADXL345_REG_POWER_CTL           (0x2D)    // Power-saving features control
    #define ADXL345_REG_INT_ENABLE          (0x2E)    // Interrupt enable control
    #define ADXL345_REG_INT_MAP             (0x2F)    // Interrupt mapping control
    #define ADXL345_REG_INT_SOURCE          (0x30)    // Source of interrupts
    #define ADXL345_REG_DATA_FORMAT         (0x31)    // Data format control
    #define ADXL345_REG_DATAX0              (0x32)    // X-axis data 0
    #define ADXL345_REG_DATAX1              (0x33)    // X-axis data 1
    #define ADXL345_REG_DATAY0              (0x34)    // Y-axis data 0
    #define ADXL345_REG_DATAY1              (0x35)    // Y-axis data 1
    #define ADXL345_REG_DATAZ0              (0x36)    // Z-axis data 0
    #define ADXL345_REG_DATAZ1              (0x37)    // Z-axis data 1
    #define ADXL345_REG_FIFO_CTL            (0x38)    // FIFO control
    #define ADXL345_REG_FIFO_STATUS         (0x39)    // FIFO status
/*=========================================================================*/

/*=========================================================================
    Bit masks for control registers
    -----------------------------------------------------------------------*/
    #define ADXL345_BIT_ACT_ACDC            (0x80)	// Activity monitor AC/DC select (1 = AC, 0 = DC)
    #define ADXL345_BIT_ACT_X               (0x40)	// Activity monitor X-axis enable
    #define ADXL345_BIT_ACT_Y               (0x20)	// Activity monitor Y-axis enable
    #define ADXL345_BIT_ACT_Z               (0x10)	// Activity monitor Z-axis enable
    #define ADXL345_BIT_INACT_ACDC          (0x08)	// Inactivity monitor AC/DC select (1 = AC, 0 = DC)
    #define ADXL345_BIT_INACT_X             (0x04)	// Inactivity monitor X-axis enable
    #define ADXL345_BIT_INACT_Y             (0x02)	// Inactivity monitor Y-axis enable
    #define ADXL345_BIT_INACT_Z             (0x01)	// Inactivity monitor Z-axis enable
    #define ADXL345_BIT_TAP_SUPPRESS        (0x80)	// Double tap suppression bit
    #define ADXL345_BIT_TAP_X               (0x40)	// Tap X-axis enable 
    #define ADXL345_BIT_TAP_Y               (0x20)	// Tap Y-axis enable 
    #define ADXL345_BIT_TAP_Z               (0x10)	// Tap Z-axis enable 
    #define ADXL345_BIT_TAP_SRC_ACT_X       (0x40)	// True if source of last activity was primarily on the X axis
    #define ADXL345_BIT_TAP_SRC_ACT_Y       (0x20)	// True if source of last activity was primarily on the X axis
    #define ADXL345_BIT_TAP_SRC_ACT_Z       (0x10)	// True if source of last activity was primarily on the X axis
    #define ADXL345_BIT_TAP_SRC_SLEEP       (0x08)	// True if the part is asleep
    #define ADXL345_BIT_TAP_SRC_TAP_X       (0x04)	// True if source of last tap was primarily on the X axis
    #define ADXL345_BIT_TAP_SRC_TAP_Y       (0x02)	// True if source of last tap was primarily on the Y axis
    #define ADXL345_BIT_TAP_SRC_TAP_Z       (0x01)	// True if source of last tap was primarily on the Z axis
    #define ADXL345_BIT_PWR_LINK            (0x20)	// Link activity and inactivity interrupts
    #define ADXL345_BIT_PWR_AUTOSLEEP       (0x10)	// Activate auto-sleep function. See p. 25 of ADXL345 datasheet
    #define ADXL345_BIT_PWR_MEASURE         (0x08)	// Turns on measurement mode
    #define ADXL345_BIT_PWR_SLEEP           (0x04)	// Turns on sleep mode
    #define ADXL345_BIT_PWR_WAKEUP_MASK     (0x03)	// Mask for setting wakeup bits. See p. 26 of ADXL345 datasheet
    #define ADXL345_BIT_INT_DATA_READY      (0x80)	// Data ready interrupt
    #define ADXL345_BIT_INT_SINGLE_TAP      (0x40)	// Single tap interrupt
    #define ADXL345_BIT_INT_DOUBLE_TAP      (0x20)	// Double tap interrupt
    #define ADXL345_BIT_INT_ACT             (0x10)	// Activity interrupt
    #define ADXL345_BIT_INT_INACT           (0x08)	// Inactivity interrupt
    #define ADXL345_BIT_INT_FREEFALL        (0x04)	// Free fall interrupt
    #define ADXL345_BIT_INT_WATERMARK       (0x02)	// Watermark interrupt
    #define ADXL345_BIT_INT_OVERRUN         (0x01)	// FIFO overrun interrupt
    #define ADXL345_BIT_FORMAT_SELF_TEST    (0x80)	// Apply self-test force
    #define ADXL345_BIT_FORMAT_SPI          (0x40)	// Select 3-wire (1) or 4-wire (0) SPI mode
    #define ADXL345_BIT_FORMAT_INT_INVERT   (0x20)	// Set interrupts active low
    #define ADXL345_BIT_FORMAT_FULL_RES     (0x80)	// Select full resolution (1) or 10-bit (0) mode
    #define ADXL345_BIT_FORMAT_JUSTIFY      (0x40)	// Select left or right justified data mode
    #define ADXL345_BIT_FORMAT_RANGE_MASK   (0x03)	// Range selection bits. Use type range_t. See p. 27 of ADXL345 datasheet
    #define ADXL345_BIT_FIFO_CTL_MODE_MASK  (0xc0)	// Set the FIFO mode. See p. 27 of ADXL345 datasheet
    #define ADXL345_BIT_FIFO_CTL_TRIGGER    (0x20)	// Trigger FIFO from INT1 (0) or INT2 (1)
    #define ADXL345_BIT_FIFO_CTL_SAMPLES_MASK (0x1f)	// Set number of samples in the FIFO
    #define ADXL345_BIT_FIFO_STAT_TRIGGER   (0xc0)	// FIFO trigger status
    #define ADXL345_BIT_FIFO_STAT_ENTRIES   (0x3f)	// Number of entries in the FIFO + output buffer
    
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define ADXL345_MG2G_MULTIPLIER (0.004)  // 4mg per lsb
/*=========================================================================*/

/* Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth */
typedef enum
{
  ADXL345_DATARATE_3200_HZ    = 0b1111, // 1600Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_1600_HZ    = 0b1110, //  800Hz Bandwidth    90µA IDD
  ADXL345_DATARATE_800_HZ     = 0b1101, //  400Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_400_HZ     = 0b1100, //  200Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_200_HZ     = 0b1011, //  100Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_100_HZ     = 0b1010, //   50Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_50_HZ      = 0b1001, //   25Hz Bandwidth    90µA IDD
  ADXL345_DATARATE_25_HZ      = 0b1000, // 12.5Hz Bandwidth    60µA IDD
  ADXL345_DATARATE_12_5_HZ    = 0b0111, // 6.25Hz Bandwidth    50µA IDD
  ADXL345_DATARATE_6_25HZ     = 0b0110, // 3.13Hz Bandwidth    45µA IDD
  ADXL345_DATARATE_3_13_HZ    = 0b0101, // 1.56Hz Bandwidth    40µA IDD
  ADXL345_DATARATE_1_56_HZ    = 0b0100, // 0.78Hz Bandwidth    34µA IDD
  ADXL345_DATARATE_0_78_HZ    = 0b0011, // 0.39Hz Bandwidth    23µA IDD
  ADXL345_DATARATE_0_39_HZ    = 0b0010, // 0.20Hz Bandwidth    23µA IDD
  ADXL345_DATARATE_0_20_HZ    = 0b0001, // 0.10Hz Bandwidth    23µA IDD
  ADXL345_DATARATE_0_10_HZ    = 0b0000  // 0.05Hz Bandwidth    23µA IDD (default value)
} dataRate_t;

/* Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range */
typedef enum
{
  ADXL345_RANGE_16_G          = 0b11,   // +/- 16g
  ADXL345_RANGE_8_G           = 0b10,   // +/- 8g
  ADXL345_RANGE_4_G           = 0b01,   // +/- 4g
  ADXL345_RANGE_2_G           = 0b00    // +/- 2g (default value)
} range_t;

class Adafruit_ADXL345 : public Adafruit_Sensor {
 public:
  Adafruit_ADXL345(int32_t sensorID = -1);
  
  bool       begin(void);
  void       setRange(range_t range);
  range_t    getRange(void);
  void       setDataRate(dataRate_t dataRate);
  dataRate_t getDataRate(void);
  void       getEvent(sensors_event_t*);
  void       getSensor(sensor_t*);
  // functions to set configuration registers
  void       setTapThreshold(float acceleration);
  void       setTapDuration(float milli);
  void       setTapLatentcy(int milli);
  void       setTapWindow(int milli);
  void       setActivityThreshold(float acceleration);
  void       setInactivityThreshold(float acceleration);
  void       setInactivityTime(uint8_t inActTime);
  void       setInactivityControl(uint8_t inactSettings);
  void       setFreefallThreshold(float acceleration);
  void       setFreefallTime(int milli);
  void       setTapAxes(uint8_t tapAxes);
  void       setPwrCtl(uint8_t pwrCtl);
  void       setIntEnable(uint8_t intEnable);
  void       setIntMap(uint8_t intMap);
  void       setDataFormat(uint8_t dataFormat);
  void       setFIFOctl(uint8_t FIFOctl);
  // functions to read configuration registers
  float      getTapThreshold(void);
  float      getTapDuration(void);
  int        getTapLatentcy(void);-
  int        getTapWindow(void);
  float      getActivityThreshold(void);
  float      getInactivityThreshold(void);
  uint8_t    getInactivityTime(void);
  uint8_t    getInactivityControl(void);
  float      getFreefallThreshold(void);
  int        getFreefallTime(void);
  uint8_t    getTapAxes(void);
  uint8_t    getPwrCtl(void);
  uint8_t    getIntEnable(void);
  uint8_t    getIntMap(void);
  uint8_t    getDataFormat(void);
  uint8_t    getFIFOctl(void);
  // functions to read status registers
  uint8_t    getTapActAxes(void);
  uint8_t    getInterruptSource(void);
  uint8_t    getFIFOstatus(void);
  // debugging functions
  void       dumpRegistersRaw(void);

 private:
  int32_t _sensorID;
};
