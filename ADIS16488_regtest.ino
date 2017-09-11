////////////////////////////////////////////////////////////////////////////////////////////////////////
//  September 2016
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16488_Teensy_BurstRead_Example.ino
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This Arduino project interfaces with an ADIS16488 using SPI and the 
//  accompanying C++ libraries, reads IMU data in LSBs, scales the data, and 
//  outputs measurements to a serial debug terminal (PuTTY) via the onboard 
//  USB serial port.
//
//  This project has been tested on a PJRC 32-Bit Teensy 3.2 Development Board, 
//  but should be compatible with any other embedded platform with some modification.
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
//  Pinout for a Teensy 3.2 Development Board
//  RST = D6
//  SCK = D13/SCK
//  CS = D10/CS
//  DOUT(MISO) = D12/MISO
//  DIN(MOSI) = D11/MOSI
//  DR = D2
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "ADIS16488.h"
#include <SPI.h>
#include <cmath>
#include <cstring>
#include "crc.h"

// Uncomment to enable debug
//#define DEBUG

const int dbgpin = 3;
const int ledpin = 6;

//const float gdelta_scale = 0.022; // 720.0/(2^15);
//const float gscale = 450.0/pow(2,31);
const float imu_dt = 1.0/(2460/4);

// Initialize Variables
// Temporary Data Array
ImuDataRaw *imuDataRaw;
ImuData *imuData;

// Accelerometer
float AXS, AYS, AZS = 0;

// Gyro
float GXS, GYS, GZS = 0;
volatile float GINTX, GINTY, GINTZ=0;
volatile float gdxraw, gdyraw, gdzraw=0;
volatile float GDELTAXS, GDELTAYS, GDELTAZS = 0;

// Magnetometer
float MXS, MYS, MZS = 0;

// Barometer
float BAROS = 0;

// Control registers
int MSC = 0;
int SENS = 0;
int SMPL = 0;

// Temperature
float TEMPS = 0;

// Struct used to send serial data
typedef struct SerialOutMsg {
    SerialOutMsg(): h1(0x42), h2(0x43), h3(0x44), h4(0x45){} // magic init by Mato
    
    uint8_t h1, h2, h3, h4; // header 0x42 0x43 0x44 0x45
    uint32_t count;         // packet counter
    float gx, gy, gz;       // gyro rate xyz
    float gintx, ginty, gintz; // gyro integrated angle xyz   
    float ax, ay, az;       // accelerometer xyz
    float mx, my, mz;       // magnetometer xyz
    float baro, temp;       // barometric pressure, temperature
} __attribute__((packed)) SerialOutMsg;

SerialOutMsg outMsg; //= {0};

// Delay counter variable
int printCounter = 0;

// Call ADIS16488 Class
ADIS16488 IMU(10,2,6); // Chip Select, Data Ready, Reset Pin Assignments

uint32_t t_start, t_prev, t_bias;
bool bias_set = false;

void setup()
{
    Serial.begin(115200); // Initialize serial output via USB
    pinMode(dbgpin, OUTPUT);
    pinMode(ledpin, OUTPUT);
    digitalWrite(ledpin, LOW);
    IMU.configSPI(); // Configure SPI communication
    delay(1000); // Give the part time to start up

    // Configure IMU settings
    IMUconfig();

    t_start = millis();
    t_prev=t_start;
    t_bias=t_start;
    
    outMsg.count=0;

//    Serial.println(sizeof(SerialOutMsg));
//    delay(2000);
    
    attachInterrupt(2, grabData, RISING); // Attach interrupt to pin 2. Trigger on the rising edge
//    int i=0;
//    while(1){
//      delay(100);
////      IMU.regWrite(PAGE_ID,0);
////      Serial.print(String(i++)+": "+IMU.regRead(PAGE_ID)+" ");
////      delay(1);
////      IMU.regWrite(PAGE_ID,3);
////      Serial.println(IMU.regRead(PAGE_ID));
//      Serial.println(IMU.regRead32(XGYRO_OUT));
//      //Serial.println(String(i++)+": "+IMU.regRead(PROD_ID)+" "+IMU.regRead(XGYRO_OUT));
//    }
}

void IMUconfig(){
    // Set decimation rate to 4 -> 616Hz
    IMU.regWrite(PAGE_ID, 3); // turn to page 3
    IMU.regWrite(DEC_RATE, 0x03); // set DEC_RATE to 3 -> decimation = 3+1=4
    delay(20);
//    Serial.println(IMU.regRead(0x0E),HEX);
//    delay(2000);
    IMU.regWrite(PAGE_ID, 0); // turn to page 0 to continue normal operation
    delay(20);
}

void resetIntAngles(){
    GINTX = 0; GINTY = 0; GINTZ = 0; 
    GDELTAXS = 0; GDELTAYS = 0; GDELTAZS = 0;
}

volatile uint32_t grabcnt = 0;
volatile int ledcnt = 0;
// Function used to read register values when an ISR is triggered using the IMU's DataReady output
void grabData()
{
    detachInterrupt(2);
    digitalWrite(dbgpin,HIGH);
    grabcnt++;
    IMU.configSPI(); // Configure SPI before the read. Useful when talking to multiple SPI devices
    imuDataRaw = IMU.readAll();
    digitalWrite(dbgpin,LOW);
    digitalWrite(dbgpin,HIGH);
    imuData = IMU.scaleData(imuDataRaw);
    digitalWrite(dbgpin,LOW);
    digitalWrite(dbgpin,HIGH);
//    gdxraw += *(burstData + 12);
//    gdyraw += *(burstData + 13);
//    gdzraw += *(burstData + 14);
//    GINTX += IMU.gyroScale(*(burstData + 1))*imu_dt; //Scale X Gyro
//    GINTY += IMU.gyroScale(*(burstData + 2))*imu_dt; //Scale Y Gyro
//    GINTZ += IMU.gyroScale(*(burstData + 3))*imu_dt; //Scale Z Gyro
//    GDELTAXS += *(burstData + 12)*gdelta_scale;
//    GDELTAYS += *(burstData + 13)*gdelta_scale;
//    GDELTAZS += *(burstData + 14)*gdelta_scale;

    GINTX += imuData->gx*imu_dt; //Scale X Gyro
    GINTY += imuData->gy*imu_dt; //Scale Y Gyro
    GINTZ += imuData->gz*imu_dt; //Scale Z Gyro
    GDELTAXS += imuData->gdx; //*(burstData + 12)*gdelta_scale;
    GDELTAYS += imuData->gdy; //*(burstData + 13)*gdelta_scale;
    GDELTAZS += imuData->gdz; //*(burstData + 14)*gdelta_scale;

    digitalWrite(dbgpin,LOW);
    if(++ledcnt == 100){
        ledcnt = 0;
        digitalWrite(ledpin, !digitalRead(ledpin));
    }
    attachInterrupt(2,grabData,RISING);
}

// Main loop. Print data to the serial port. Sensor sampling is performed in the ISR
void loop() {   
    //uint32_t t_start = millis();
    //IMU.configSPI(); // Configure SPI before the read. Useful when talking to multiple SPI devices
    //burstData = IMU.burstRead(); // Read data and insert into array
    //burstData = IMU.readAll();

    detachInterrupt(2); //Detach interrupt to avoid overwriting data
    uint32_t t_now = millis();
    uint32_t dt = t_now-t_prev;
    if(dt>500){
        //Serial.println(grabcnt);
//        Serial.println(1000*grabcnt/(float)dt);
        grabcnt=0;
        t_prev = t_now;
    }

    if(Serial.available()>2){
        uint8_t b1 = Serial.read();
        uint8_t b2 = Serial.read();
        uint8_t cmd = Serial.read();
        //Serial.println(String(b1)+" "+b2+" "+cmd);
        if(b1==50 && b2==87){
            switch(cmd){
              case 139: // Trigger gyro offset calibration
                  digitalWrite(ledpin,HIGH);
                  IMU.calibrateBiasNull();
                  delay(2000);
                  digitalWrite(ledpin,LOW);
                  break;
              case 140: // Reset the IMU
                  digitalWrite(ledpin,HIGH);
                  IMU.swReset();
                  delay(100);
                  IMUconfig();
                  t_bias = t_now;
                  digitalWrite(ledpin,LOW);
                  break;
              case 141: // Reset integrated gyro angles
                  digitalWrite(ledpin,HIGH);
                  resetIntAngles();
                  digitalWrite(ledpin,LOW);
                  break;
              default:
                break;
            }
        }
    }

    outMsg.count++;
    outMsg.gx = imuData->gx; 
    outMsg.gy = imuData->gy; 
    outMsg.gz = imuData->gz;
    outMsg.gintx = GDELTAXS; 
    outMsg.ginty = GDELTAYS; 
    outMsg.gintz = GDELTAZS;
    outMsg.ax = imuData->ax; 
    outMsg.ay = imuData->ay; 
    outMsg.az = imuData->az;
    outMsg.mx = imuData->mx; 
    outMsg.my = imuData->my; 
    outMsg.mz = imuData->mz;
    outMsg.baro = imuData->baro;
    outMsg.temp = imuData->temp;
    uint8_t crc = computeCRC((uint8_t*)&outMsg, sizeof(SerialOutMsg));
    
//    Se
    Serial.write(sizeof(SerialOutMsg));
    Serial.write((char*)&outMsg, sizeof(SerialOutMsg));
    Serial.write(crc);
    Serial.write(0x51);
    Serial.write(0x52);
    Serial.write(0x53);
    Serial.write(0x54);
//    Serial.write(0x44);
//    Serial.write('\n');
    
//    if(t_now-t_bias>30000 && !bias_set){
//        IMU.regWrite(PAGE_ID, 3); // turn to page 3
//        IMU.regWrite(GLOB_CMD, 0x01); // set Bias null bit in GLOB_CMD register
//        delay(20);
//    //    Serial.println(IMU.regRead(0x0E),HEX);
//    //    delay(2000);
//        IMU.regWrite(PAGE_ID, 0); // turn to page 0 to continue normal operation
//        delay(20);
//        bias_set=true;
//        t_bias=t_now;
//        Serial.println("bias null");
//    }

//    Serial.print(imuData->gx,4); Serial.print(",");
//    Serial.print(imuData->gy,4); Serial.print(",");
//    Serial.print(imuData->gz,4); Serial.print(",");
//    Serial.print(imuData->ax,4); Serial.print(",");
//    Serial.print(imuData->ay,4); Serial.print(",");
//    Serial.print(imuData->az,4); Serial.print(",");
//    Serial.print(imuData->mx); Serial.print(",");
//    Serial.print(imuData->my); Serial.print(",");
//    Serial.print(imuData->mz); Serial.print(",");
//    Serial.print(imuData->temp); Serial.print(",");
    
//    Serial.print(GXS); Serial.print(",");
//    Serial.print(GYS); Serial.print(",");
//    Serial.print(GZS); Serial.print(",");

//      Serial.print(gdxraw); Serial.print(",");
//      Serial.print(gdyraw); Serial.print(",");
//      Serial.print(gdzraw); Serial.print(",");
//      Serial.print((*(burstData + 12))); Serial.print(",");
//      Serial.print((*(burstData + 13))); Serial.print(",");
//      Serial.print((*(burstData + 14))); Serial.print(",");
//    Serial.print(GDELTAXS,4); Serial.print(",");
//    Serial.print(GDELTAYS,4); Serial.print(",");
//    Serial.print(GDELTAZS,4); Serial.print(",");
//    Serial.print(GINTX,4); Serial.print(",");
//    Serial.print(GINTY); Serial.print(",");
//    Serial.print(GINTZ); Serial.print(",");

//    Serial.print(AXS); Serial.print(",");
//    Serial.print(AYS); Serial.print(",");
//    Serial.print(AZS); Serial.print(",");
//    Serial.print(MXS); Serial.print(",");
//    Serial.print(MYS); Serial.print(",");
//    Serial.print(MZS); Serial.print(",");
//    Serial.print(BAROS); Serial.print(",");
////    Serial.print((*(burstData + 7))); Serial.print(","); // temperature test
//    Serial.print(TEMPS); //Serial.print(",");

    if(t_now-t_bias>30000 && IMU.getLastCalibTime()==0){
        IMU.calibrateBiasNull();
        t_bias=t_now;
        resetIntAngles();
        Serial.print("calib bias null start");
    }

      
    Serial.println();
    attachInterrupt(2, grabData, RISING);
    delay(50);
}
