/**************************************************************************/
/*!
    @file     Adafruit_ADXL345.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    The ADXL345 is a digital accelerometer with 13-bit resolution, capable
    of measuring up to +/-16g.  This driver communicate using I2C.

    This is a library for the Adafruit ADXL345 breakout
    ----> https://www.adafruit.com/products/???

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

	  v1.1 - Added Adafruit_Sensor library support
    v1.0 - First release
*/
/**************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include <limits.h>

#include "Adafruit_ADXL345.h"

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
static uint8_t i2cread(void) {
  #if ARDUINO >= 100
  return Wire.read();
  #else
  return Wire.receive();
  #endif
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
static void i2cwrite(uint8_t x) {
  #if ARDUINO >= 100
  Wire.write((uint8_t)x);
  #else
  Wire.send(x);
  #endif
}

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
static void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  i2cwrite((uint8_t)reg);
  i2cwrite((uint8_t)(value));
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
static uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  i2cwrite(reg);
  Wire.endTransmission();
  Wire.requestFrom(ADXL345_ADDRESS, 1);
  return (i2cread());  
}

/**************************************************************************/
/*!
    @brief  Reads 16-bits from the specified register
*/
/**************************************************************************/
static int16_t read16(uint8_t reg) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  i2cwrite(reg);
  Wire.endTransmission();
  Wire.requestFrom(ADXL345_ADDRESS, 2);
  return (uint16_t)(i2cread() | (i2cread() << 8));  
}

/**************************************************************************/
/*! 
    @brief  Read the device ID (can be used to check connection)
*/
/**************************************************************************/
static uint8_t getDeviceID(void) {
  // Check device ID register
  return readRegister(ADXL345_REG_DEVID);
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent X axis value
*/
/**************************************************************************/
static int16_t getX(void) {
  return read16(ADXL345_REG_DATAX0);
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent Y axis value
*/
/**************************************************************************/
static int16_t getY(void) {
  return read16(ADXL345_REG_DATAY0);
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent Z axis value
*/
/**************************************************************************/
static int16_t getZ(void) {
  return read16(ADXL345_REG_DATAZ0);
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL345 class
*/
/**************************************************************************/
Adafruit_ADXL345::Adafruit_ADXL345(int32_t sensorID) {
  _sensorID = sensorID;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool Adafruit_ADXL345::begin() {
  Wire.begin();

  /* Check connection */
  uint8_t deviceid = getDeviceID();
  if (deviceid != 0xE5)
  {
    /* No ADXL345 detected ... return false */
    Serial.println(deviceid, HEX);
    return false;
  }
  
  // Enable measurements
  writeRegister(ADXL345_REG_POWER_CTL, 0x08);  
  
  return true;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void Adafruit_ADXL345::setRange(range_t range)
{
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister(ADXL345_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_DATA_FORMAT, format);
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
range_t Adafruit_ADXL345::getRange(void)
{
  /* Red the data format register to preserve bits */
  return (range_t)(readRegister(ADXL345_REG_DATA_FORMAT) & 0x03);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
void Adafruit_ADXL345::setDataRate(dataRate_t dataRate)
{
  /* Note: The LOW_POWER bits are currently ignore and we always keep
     the device in 'normal' mode */
  writeRegister(ADXL345_REG_BW_RATE, dataRate);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
dataRate_t Adafruit_ADXL345::getDataRate(void)
{
  return (dataRate_t)(readRegister(ADXL345_REG_BW_RATE) & 0x0F);
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
void Adafruit_ADXL345::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;
  event->acceleration.x = getX() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = getY() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = getZ() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
}

/**************************************************************************/
/*! 
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Adafruit_ADXL345::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "ADXL345", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_PRESSURE;
  sensor->min_delay   = 0;
  sensor->max_value   = 300.0F;               // 300..1100 hPa
  sensor->min_value   = 1100.0F;
  sensor->resolution  = 0.01F;                // 0.01 hPa resolution
}

/**************************************************************************/
/*! 
    @brief  Set the threshold for tap detection, in gees
*/
/**************************************************************************/

void Adafruit_ADXL345::setTapThreshold(float acceleration) {
  if (acceleration > 16) acceleration = 16;
  if (acceleration < 0) acceleration = 0;
  writeRegister(ADXL345_REG_THRESH_TAP, (uint8_t)(int(acceleration*16)));
  return;
}

/**************************************************************************/
/*! 
    @brief  Get the threshold for tap detection, in gees
*/
/**************************************************************************/

float Adafruit_ADXL345::getTapThreshold(void) {
  return (float(readRegister(ADXL345_REG_THRESH_TAP)) * 0.0625);
}

/**************************************************************************/
/*! 
    @brief  Set the maximum duration for a tap, in milliseconds
*/
/**************************************************************************/

void Adafruit_ADXL345::setTapDuration(float milli) {
  if (milli > 159.375) milli = 159.375;
  if (milli < 0) milli = 0;
  writeRegister(ADXL345_REG_DUR, (uint8_t)(int(milli*1.6)));
  return;
}

/**************************************************************************/
/*! 
    @brief  Get the maximum duration for a tap, in milliseconds
*/
/**************************************************************************/

float Adafruit_ADXL345::getTapDuration(void) {
  return(float(readRegister(ADXL345_REG_DUR)) * 0.625);
}

/**************************************************************************/
/*! 
    @brief  Set the latency between the end of one tap and the window for a possible second tap, in milliseconds
*/
/**************************************************************************/

void Adafruit_ADXL345::setTapLatentcy(int milli) {
  if (milli > 318.75) milli = 318.75;
  if (milli < 0) milli = 0;
  writeRegister(ADXL345_REG_LATENT, (uint8_t)(int(milli*0.8)));
  return;
}

/**************************************************************************/
/*! 
    @brief  Get the latency between the end of one tap and the window for a possible second tap, in milliseconds
*/
/**************************************************************************/

int Adafruit_ADXL345::getTapLatentcy(void) {
  return(int(readRegister(ADXL345_REG_LATENT) * 1.25));
}

/**************************************************************************/
/*! 
    @brief  Set the detection window for a possible second tap, in milliseconds
*/
/**************************************************************************/

void Adafruit_ADXL345::setTapWindow(int milli) {
  if (milli > 318.75) milli = 318.75;
  if (milli < 0) milli = 0;
  writeRegister(ADXL345_REG_WINDOW, (uint8_t)(int(milli*0.8)));
  return;
}

/**************************************************************************/
/*! 
    @brief  Get the detection window for a possible second tap, in milliseconds
*/
/**************************************************************************/

int Adafruit_ADXL345::setTapWindow(void) {
  return(int(readRegister(ADXL345_REG_WINDOW) * 1.25));
}

/**************************************************************************/
/*! 
    @brief  Set the minimum acceleration that trips an activity interrupt, in gees
*/
/**************************************************************************/

void Adafruit_ADXL345::setActivityThreshold(float acceleration) {
  if (acceleration > 16) acceleration = 16;
  if (acceleration < 0) acceleration = 0;
  writeRegister(ADXL345_REG_THRESH_ACT, (uint8_t)(int(acceleration*16)));
  return;
}

/**************************************************************************/
/*! 
    @brief  Get the minimum acceleration that trips an activity interrupt, in gees
*/
/**************************************************************************/

float Adafruit_ADXL345::getActivityThreshold(void) {
  return(float(readRegister(ADXL345_REG_THRESH_ACT)) * 0.0625);
}

/**************************************************************************/
/*! 
    @brief  Set the maximum acceleration that trips an inactivity interrupt, in gees
*/
/**************************************************************************/

void Adafruit_ADXL345::setInactivityThreshold(float acceleration) {
  if (acceleration > 16) acceleration = 16;
  if (acceleration < 0) acceleration = 0;
  writeRegister(ADXL345_REG_THRESH_INACT, (uint8_t)(int(acceleration*16)));
  return;
}

/**************************************************************************/
/*! 
    @brief  Get the maximum acceleration that trips an inactivity interrupt, in gees
*/
/**************************************************************************/

float Adafruit_ADXL345::getInactivityThreshold(void) {
  return(float(readRegister(ADXL345_REG_THRESH_INACT)) * 0.0625);
}

/**************************************************************************/
/*! 
    @brief  Set the time delay required to trip an inactivity interrupt in seconds
*/
/**************************************************************************/

void Adafruit_ADXL345::setInactivityTime(uint8_t inActTime) {
  writeRegister(ADXL345_REG_TIME_INACT, inActSettings);
  return;
}

/**************************************************************************/
/*! 
    @brief  Get the time delay required to trip an inactivity interrupt in seconds
*/
/**************************************************************************/

uint8_t Adafruit_ADXL345::getInactivityTime(void) {
  return(readRegister(ADXL345_REG_TIME_INACT));
}

/**************************************************************************/
/*! 
    @brief  Set the control bits for activity/inactivity interrupts
*/
/**************************************************************************/

void Adafruit_ADXL345::setInactivityControl(uint8_t inActSettings) {
  writeRegister(ADXL345_REG_ACT_INACT_CTL, inActSettings);
  return;
}

/**************************************************************************/
/*! 
    @brief  Get the control bits for activity/inactivity interrupts
*/
/**************************************************************************/

uint8_t Adafruit_ADXL345::getInactivityControl(void) {
  return(readRegister(ADXL345_REG_ACT_INACT_CTL));
}

/**************************************************************************/
/*! 
    @brief  Set the maximum acceleration that trips a free-fall interrupt, in gees
*/
/**************************************************************************/

void Adafruit_ADXL345::setFreefallThreshold(float acceleration) {
  if (acceleration > 16) acceleration = 16;
  if (acceleration < 0) acceleration = 0;
  writeRegister(ADXL345_REG_THRESH_FF, (uint8_t)(int(acceleration*16)));
  return;  
}

/**************************************************************************/
/*! 
    @brief  Get the maximum acceleration that trips a free-fall interrupt, in gees
*/
/**************************************************************************/

float Adafruit_ADXL345::getFreefallThreshold(void) {
  return(float(readRegister(ADXL345_REG_THRESH_FF)) * 0.0625);
}

/**************************************************************************/
/*! 
    @brief  Set the minimum free fall time to trip an interrupt, in milliseconds
*/
/**************************************************************************/

void Adafruit_ADXL345::setFreefallTime(int milli) {
  if (milli > 1275) milli = 1275;
  if (milli < 0) milli = 0;
  writeRegister(ADXL345_REG_TIME_FF, (uint8_t)(int(milli/5)));
  return;
}

/**************************************************************************/
/*! 
    @brief  Get the minimum free fall time to trip an interrupt, in milliseconds
*/
/**************************************************************************/

int Adafruit_ADXL345::getFreefallTime(void) {
  return(readRegister(ADXL345_REG_TIME_FF) * 5);
}

/**************************************************************************/
/*! 
    @brief  Set the axes that taps are detected on
*/
/**************************************************************************/

void Adafruit_ADXL345::setTapAxes(uint8_t tapAxes) {
  writeRegister(ADXL345_REG_TAP_AXES, tapAxes);
  return;
}

/**************************************************************************/
/*! 
    @brief  Get the axes that taps are detected on
*/
/**************************************************************************/

void Adafruit_ADXL345::getTapAxes(uint8_t tapAxes) {
  return(readRegister(ADXL345_REG_TAP_AXES));
}

/**************************************************************************/
/*! 
    @brief  Set power control register
*/
/**************************************************************************/

void Adafruit_ADXL345::setPwrCtl(uint8_t pwrCtl) {
  writeRegister(ADXL345_REG_POWER_CTL, pwrCtl);
  return;
}

/**************************************************************************/
/*! 
    @brief  Get power control register
*/
/**************************************************************************/

void Adafruit_ADXL345::getPwrCtl(uint8_t pwrCtl) {
  return(readRegister(ADXL345_REG_POWER_CTL));
}

/**************************************************************************/
/*! 
    @brief  Set interrupt enable register
*/
/**************************************************************************/

void Adafruit_ADXL345::setIntEnable(uint8_t intEnable) {
  writeRegister(ADXL345_REG_INT_ENABLE, intEnable);
  return;
}

/**************************************************************************/
/*! 
    @brief  Get interrupt enable register
*/
/**************************************************************************/

void Adafruit_ADXL345::getIntEnable(uint8_t intEnable) {
  return(readRegister(ADXL345_REG_INT_ENABLE));
}

/**************************************************************************/
/*! 
    @brief  Set interrupt map register
*/
/**************************************************************************/

void Adafruit_ADXL345::setIntMap(uint8_t intMap) {
  writeRegister(ADXL345_REG_INT_MAP, intMap);
  return;
}

/**************************************************************************/
/*! 
    @brief  Get interrupt map register
*/
/**************************************************************************/

void Adafruit_ADXL345::getIntMap(uint8_t intMap) {
  return(readRegister(ADXL345_REG_INT_MAP));
}

/**************************************************************************/
/*! 
    @brief  Set data format register
*/
/**************************************************************************/

void Adafruit_ADXL345::setDataFormat(uint8_t dataFormat) {
  writeRegister(ADXL345_REG_DATA_FORMAT, dataFormat);
  return;
}

/**************************************************************************/
/*! 
    @brief  Get data format register
*/
/**************************************************************************/

void Adafruit_ADXL345::getDataFormat(uint8_t dataFormat) {
  return(readRegister(ADXL345_REG_DATA_FORMAT));
}

/**************************************************************************/
/*! 
    @brief  Set FIFO configuration register
*/
/**************************************************************************/

void Adafruit_ADXL345::setFIFOctl(uint8_t FIFOctl) {
  writeRegister(ADXL345_REG_FIFO_CTL, FIFOctl);
  return;
}

/**************************************************************************/
/*! 
    @brief  Get FIFO configuration register
*/
/**************************************************************************/

void Adafruit_ADXL345::getFIFOctl(uint8_t FIFOctl) {
  return(readRegister(ADXL345_REG_FIFO_CTL));
}

/**************************************************************************/
/*! 
    @brief  Get tap & activity source register
*/
/**************************************************************************/

uint8_t Adafruit_ADXL345::getTapActAxes(void) {
  return(readRegister(ADXL345_REG_ACT_TAP_STATUS));
}

/**************************************************************************/
/*! 
    @brief  Get interrupt source register
*/
/**************************************************************************/

uint8_t Adafruit_ADXL345::getInterruptSource(void) {
  return(readRegister(ADXL345_REG_INT_SOURCE));
}

/**************************************************************************/
/*! 
    @brief  Get FIFO status register
*/
/**************************************************************************/

uint8_t Adafruit_ADXL345::getFIFOstatus(void) {
  return(readRegister(ADXL345_REG_FIFO_STATUS));
}

/**************************************************************************/
/*! 
    @brief  Dump raw value of all registers to Serial
*/
/**************************************************************************/

void    Adafruit_ADXL345::dumpRegistersRaw(void) {
    Serial.print("Device ID\t0x"); Serial.println(readRegister(ADXL345_REG_DEVID), HEX);
    Serial.print("Tap Threshold\t0x"); Serial.println(readRegister(ADXL345_REG_THRESH_TAP), HEX);
    Serial.print("X offset\t0x"); Serial.println(readRegister(ADXL345_REG_OFSX), HEX);
    Serial.print("Y offset\t0x"); Serial.println(readRegister(ADXL345_REG_OFSY), HEX);
    Serial.print("Z offset\t0x"); Serial.println(readRegister(ADXL345_REG_OFSZ), HEX);
    Serial.print("Tap Duration\t0x"); Serial.println(readRegister(ADXL345_REG_DUR), HEX);
    Serial.print("Tap Latency\t0x"); Serial.println(readRegister(ADXL345_REG_LATENT), HEX);
    Serial.print("Tap Window\t0x"); Serial.println(readRegister(ADXL345_REG_WINDOW), HEX);
    Serial.print("Activity Threshold\t0x"); Serial.println(readRegister(ADXL345_REG_THRESH_ACT), HEX);
    Serial.print("Inactivity Threshold\t0x"); Serial.println(readRegister(ADXL345_REG_THRESH_INACT), HEX);
    Serial.print("Inactivity Timeout\t0x"); Serial.println(readRegister(ADXL345_REG_TIME_INACT), HEX);
    Serial.print("Activity/Inactivity Ctl\t0x"); Serial.println(readRegister(ADXL345_REG_ACT_INACT_CTL), HEX);
    Serial.print("Freefall Threshold\t0x"); Serial.println(readRegister(ADXL345_REG_THRESH_FF), HEX);
    Serial.print("Freefall Time\t0x"); Serial.println(readRegister(ADXL345_REG_TIME_FF), HEX);
    Serial.print("Tap Axis Control\t0x"); Serial.println(readRegister(ADXL345_REG_TAP_AXES), HEX);
    Serial.print("Activity/Tap Status\t0x"); Serial.println(readRegister(ADXL345_REG_ACT_TAP_STATUS), HEX);
    Serial.print("Data Rate\t0x"); Serial.println(readRegister(ADXL345_REG_BW_RATE), HEX);
    Serial.print("Power Control\t0x"); Serial.println(readRegister(ADXL345_REG_POWER_CTL), HEX);
    Serial.print("Interrupt Enable\t0x"); Serial.println(readRegister(ADXL345_REG_INT_ENABLE), HEX);
    Serial.print("Interrupt Map\t0x"); Serial.println(readRegister(ADXL345_REG_INT_MAP), HEX);
    Serial.print("Interrupt Source\t0x"); Serial.println(readRegister(ADXL345_REG_INT_SOURCE), HEX);
    Serial.print("Data Format\t0x"); Serial.println(readRegister(ADXL345_REG_DATA_FORMAT), HEX);
    Serial.print("X0 Data\t0x"); Serial.println(readRegister(ADXL345_REG_DATAX0), HEX);
    Serial.print("X1 Data\t0x"); Serial.println(readRegister(ADXL345_REG_DATAX1), HEX);
    Serial.print("Y0 Data\t0x"); Serial.println(readRegister(ADXL345_REG_DATAY0), HEX);
    Serial.print("Y1 Data\t0x"); Serial.println(readRegister(ADXL345_REG_DATAY1), HEX);
    Serial.print("Z0 Data\t0x"); Serial.println(readRegister(ADXL345_REG_DATAZ0), HEX);
    Serial.print("Z1 Data\t0x"); Serial.println(readRegister(ADXL345_REG_DATAZ1), HEX);
    Serial.print("FIFO Control\t0x"); Serial.println(readRegister(ADXL345_REG_FIFO_CTL), HEX);
    Serial.print("FIFO Status\t0x"); Serial.println(readRegister(ADXL345_REG_FIFO_STATUS), HEX);
}