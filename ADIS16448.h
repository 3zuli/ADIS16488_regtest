////////////////////////////////////////////////////////////////////////////////////////////////////////
//  September 2016
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16448.h
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This library provides all the functions necessary to interface the ADIS16448 IMU with a 
//  PJRC 32-Bit Teensy 3.2 Development Board. Functions for SPI configuration, reads and writes,
//  and scaling are included. This library may be used for the entire ADIS1646X family of devices 
//  with some modification.
//
//  Permission is hereby granted, free of charge, to any person obtaining
//  a copy of this software and associated documentation files (the
//  "Software"), to deal in the Software without restriction, including
//  without limitation the rights to use, copy, modify, merge, publish,
//  distribute, sublicense, and/or sell copies of the Software, and to
//  permit persons to whom the Software is furnished to do so, subject to
//  the following conditions:
//
//  The above copyright notice and this permission notice shall be
//  included in all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
//  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ADIS16448_h
#define ADIS16448_h
#include "Arduino.h"
#include <SPI.h>
#include <cmath>

#define ADIS16480_PAGE_SIZE 0x80
#define ADIS16480_REG(page, reg) ((page) * ADIS16480_PAGE_SIZE + (reg))

// User Register Memory Map from Table 6
#define PAGE_ID ADIS16480_REG(0x00,0x00)  //Flash memory write count
#define TEMP_OUT ADIS16480_REG(0x00,0x0E)  //Flash memory write count
#define XGYRO_OUT ADIS16480_REG(0x00,0x12)  //X-axis gyroscope output
#define YGYRO_OUT ADIS16480_REG(0x00,0x16)  //Y-axis gyroscope output
#define ZGYRO_OUT ADIS16480_REG(0x00,0x1A)  //Z-axis gyroscope output
#define XGYRO_DELTAANG_OUT ADIS16480_REG(0x00,0x42)  //X-axis DELTAANG gyroscope output
#define YGYRO_DELTAANG_OUT ADIS16480_REG(0x00,0x46)  //Y-axis DELTAANG gyroscope output
#define ZGYRO_DELTAANG_OUT ADIS16480_REG(0x00,0x4A)  //Z-axis DELTAANG gyroscope output
#define XACCL_OUT ADIS16480_REG(0x00,0x1E)  //X-axis accelerometer output
#define YACCL_OUT ADIS16480_REG(0x00,0x22)  //Y-axis accelerometer output
#define ZACCL_OUT ADIS16480_REG(0x00,0x26)  //Z-axis accelerometer output
#define XMAGN_OUT ADIS16480_REG(0x00,0x28)  //X-axis magnetometer output
#define YMAGN_OUT ADIS16480_REG(0x00,0x2A)  //Y-axis magnetometer output
#define ZMAGN_OUT ADIS16480_REG(0x00,0x2C)  //Z-axis magnetometer output
#define BARO_OUT ADIS16480_REG(0x00,0x30)   //Barometer pressure measurement, high word
#define TEMP_OUT ADIS16480_REG(0x00,0x0E)   //Temperature output
#define XGYRO_OFF ADIS16480_REG(0x02,0x12)  //X-axis gyroscope bias offset factor
#define YGYRO_OFF ADIS16480_REG(0x02,0x16)  //Y-axis gyroscope bias offset factor
#define ZGYRO_OFF ADIS16480_REG(0x02,0x1A)  //Z-axis gyroscope bias offset factor
#define XACCL_OFF ADIS16480_REG(0x02,0x1E)  //X-axis acceleration bias offset factor
#define YACCL_OFF ADIS16480_REG(0x02,0x22)  //Y-axis acceleration bias offset factor
#define ZACCL_OFF ADIS16480_REG(0x02,0x26)  //Z-axis acceleration bias offset factor
// // // // // 
#define XMAGN_HIC ADIS16480_REG(0x02,0x28)  //X-axis magnetometer, hard iron factor
#define YMAGN_HIC ADIS16480_REG(0x02,0x2A)  //Y-axis magnetometer, hard iron factor
#define ZMAGN_HIC ADIS16480_REG(0x02,0x2C)  //Z-axis magnetometer, hard iron factor
#define XMAGN_SIC ADIS16480_REG(0x02,0x2E)  //X-axis magnetometer, soft iron factor
#define YMAGN_SIC ADIS16480_REG(0x02,0x30)  //Y-axis magnetometer, soft iron factor
#define ZMAGN_SIC ADIS16480_REG(0x02,0x32)  //Z-axis magnetometer, soft iron factor
#define GPIO_CTRL ADIS16480_REG(0x03,0x08)  //GPIO control
#define MSC_CTRL ADIS16480_REG(0x03,0x06)   //MISC control // FNCTIO_CTRL
#define DEC_RATE ADIS16480_REG(0x03,0x0C)   //Sample clock/Decimation filter control //DEC_RATE
#define NULL_CNFG ADIS16480_REG(0x03,0x0E)  //Bias null command
//#define SENS_AVG 0x38   //Digital filter control
#define SEQ_CNT 0x3A    //MAGN_OUT and BARO_OUT counter
#define DIAG_STAT ADIS16480_REG(0x00,0x0A)  //System status
#define GLOB_CMD ADIS16480_REG(0x03,0x02)   //System command
#define ALM_MAG1 0x40   //Alarm 1 amplitude threshold
#define ALM_MAG2 0x42   //Alarm 2 amplitude threshold
#define ALM_SMPL1 0x44  //Alarm 1 sample size
#define ALM_SMPL2 0x46  //Alarm 2 sample size
#define ALM_CTRL 0x48   //Alarm control
#define LOT_ID1 0x52    //Lot identification number
#define LOT_ID2 0x54    //Lot identification number
#define PROD_ID ADIS16480_REG(0x00,0x7E)    //Product identifier
#define SERIAL_NUM ADIS16480_REG(0x04,0x20) //Lot-specific serial number

typedef struct ImuDataRaw {
  int32_t gx, gy, gz;
  int32_t gdx, gdy, gdz;    
  int32_t ax, ay, az; 
  int16_t mx, my, mz; 
  int32_t baro;
  int16_t temp;
} ImuDataRaw;

typedef struct ImuData {
  float gx, gy, gz;
  float gdx, gdy, gdz;    
  float ax, ay, az; 
  float mx, my, mz;  
  float baro, temp; 
} ImuData;

// ADIS16448 class definition
class ADIS16448 {

public:
  // Constructor with configurable CS, data ready, and HW reset pins

  // ADIS16448(int CS, int DR, int RST, int MOSI, int MISO, int CLK);
  ADIS16448(int CS, int DR, int RST);

  // Destructor
  ~ADIS16448();

  // Performs hardware reset by sending pin 8 low on the DUT for 2 seconds
  int resetDUT(uint8_t ms);

  // Performs software reset by setting Software reset bit in GLOB_CMD reg
  int swReset();

  // Sets SPI bit order, clock divider, and data mode
  int configSPI();

  // Read single register from sensor
  int16_t regRead(uint8_t regAddr);

  // Read OUT register and its corresponding LOW register from sensor as a 32bit value
  int32_t regRead32(uint8_t regAddr);

  // Write register
  int regWrite(uint8_t regAddr, int16_t regData);

  // Read sensor data using a burst read
  int16_t *burstRead(void);

  // Sequentially read all sensor data
  ImuDataRaw *readAll(void);

  ImuData *scaleData(ImuDataRaw *raw);

  void calibrateBiasNull();

  uint32_t getLastCalibTime();

  // Scale accelerator data
  float accelScale(int16_t sensorData);
  float accelScale32(int32_t sensorData);

  // Scale gyro data
  float gyroScale(int16_t sensorData);
  float gyroScale32(int32_t sensorData);

  // Scale gyro DELTAANG data
  float gyroDeltaScale(int16_t sensorData);
  float gyroDeltaScale32(int32_t sensorData);

  // Scale temperature data
  float tempScale(int16_t sensorData);

 //Scale barometer data. Returns scaled data as float.
  float pressureScale(int16_t sensorData);
  float pressureScale32(int32_t sensorData);

  //Scale magnetometer data. Returns scaled data as float.
  float magnetometerScale(int16_t sensorData);

private:
  // Variables to store hardware pin assignments
  int _CS;
  int _DR;
  int _RST;
  int _stall = 1; //20

  uint32_t t_bias = 0; // Time of last bias calibration

  // Scale factors
  const float G = 9.798; // 1G = 9.798 m.s^-2 https://ez.analog.com/docs/DOC-2755
  const float gScale    = 0.02; // 0.02deg/s/LSB
  const float gScale32  = 0.02/pow(2,16); 
  const float gDScale   = 720.0/pow(2,15); // 0.022deg/LSB
  const float gDScale32 = 720.0/pow(2,31);
  const float aScale    = 0.0008; // 0.8mg/LSB
  const float aScale32  = 0.0008/pow(2,16);
  const float mScale    = 0.1;  // (0.1 uGa/LSB)
  const float barScale  = 0.0004; //  40mbar/LSB
  const float barScale32= 1.31/pow(2,31);
};

#endif
