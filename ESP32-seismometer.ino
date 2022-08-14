/* ESP32-seismometer.ino
## Realtime ESP32-seismomter by MPU6050/MPU9250 with BMP388 barometer
  
This program is a personal experiment to measure realtime Japan Meteorological 
Agency(JMA)'s instrumental seismic intensity scale (Shindo) with simplified 
algorithm, not fully following the procedures defined in reference [2].

No warranty for the precision of the intensity scale provided by this program.
Shindo (JMA's instrumental seismic intensity scale ) is only used in Japan and it is different from  MSK or MM scale.[1]

## About Shindo (JMA's instrumental seismic intensity scale )
- Shindo 2.5-3.4 is around 6-20 gal and frequently observed in Japan. 
- Shindo 3.5-4.4 is around 20-60 gal.
- Shindo 4.5-4.9 is around 60-110 gal. Get ready to protect our life.
- Shindo 5.0-5.4 is around 110-200 gal. I have experienced this on the 14th floor in March 11th,2011.  Extremely scary earthquake!

Shindo(Instrumental seismic intensity) and acceleration is quoted from [3]
Up to Shindo 2 is a noise level by MPU6050.

## How to get Shindo (IfF)

IfF (Shindo: Instrumental seismic intensity scale by JMA ) is by definition,

         IfF = 2.0 * log10( af ) + 0.94 

where "af" is acceleration in gal,
after filtering in the frequency domain for the entire time span using specified filter by JMA [2].

Max "af" acceleration corresponds 0.3sec duration is used to calculate max IfF for the entire shake. [2]

For example , IfF[]={3.2 , 1.2 , 2.5 , 2.2 , 3.1} is observed every 100msec
, then let sort these values to {3.2,3.1,2.5,2.2,1.2} and the third value 2.5 is the max IfF Shindo for the earthquake.

## Realtime IfF

Procedure [2] should be done for the entire waveform of the acceleration vector
, but this program provides realtime IfF using a sliding buffer of 3 sec (30 samples) and 
simplified digital low pass/high pass filter in the time domein. 

Since the filter in frequency domain specified in [2] is not easy to implement,
software high pass filter at 0.2Hz and lowpass filter at 5Hz 
from sampling interval 10Hz are used to mimic the filter curve defined in [2].
High pass filter by Filters.h [5] also removes the gravity (980 gal), thanks to [4].

In this program , IfF at every 100msec are held for 3 sec in the sliding buffer
(30 samples) and sorted every 100msec to find the third highest value, corresponding to 0.3sec rule in [2]. (thanks to ArduinoSort[7])

Max IfF during the last 30 sec is latched and published to MQTT server as Json format data, expecting
Thingsboard running at MQTT port=2883 visualizes the Json data.( Note: default thingsboard opens port=1883 )
IfF_h is also calculated from horizontal acceleration.

    Json format : {"press1": 1023.12,"temp1": 12.5,"IfF": 2.51 , "IfF_h":2.31}

- press1: barometric pressure in hPa from BMP388
- temp1: temperature in C from BMP388
- IfF:  seismic intensity from 3D acceleration
- IfF_h: seismic intensity from horizontal acceleration

Acceleration is measured by MPU6050/MPU9250 placed horizontally.
Since earthquake or volcanic erruption may affect barometric pressure , BMP388 is attached.
Both devices are connected via I2C bus. If BMP388 is not attached , this program only
sends IfF and IfF_h.

## Simulation 
Simulation earthquake data BCJ-L1 and BCJ-L2 are available as an Exel format data from [8].
BCJ data is one dimensional data , simulation data are copied to X,Y and Z axis (half of X axis).
I have converted it to float bcjL1_wave[] and bcjL2_wave[] and run the simulation by #define SIMULATION
BCJ-L1,BCJ-L2 data are not included in this repository.

| data | BCJ-L1 | BCJ-L2 | comment |
|------|------------|---------|-----------------|
|Shindo| IfF=5.5    | IfF=6.0 | (Ref.[6] page.8)|
| this program | IfF=5.44 | IfF=5.93 | for 0.2Hz software high pass filter|

The result is not bad by chance, real earth quake is not yet observed.

## Hardware 
* ESP32devkit 
* GY-521 MPU6050 or MPU9250 board wired on I2C bus. 
* BMP388(optional) from Adafruit on I2C bus.
           
           SDA = GPIO_NUM_21 , SCL = GPIO_NUM_22
           LED and appropriate resister at GPIO_NUM_2 to indicate WiFi is ready

## Bluetooth Serial monitor
Bluttooth serial: acceleration(cm/s2) and IfF can be monitered by Bluetooth Serial terminal as "Seismo-BT-serial". 
Huge partition scheme in Arduino IDE is recommended when #define BTSERIAL is on.

## Development environment
Arduino board manager 1.0.6 / Arduino IDE 1.8.19



## References
  - [1] https://en.wikipedia.org/wiki/Japan_Meteorological_Agency_seismic_intensity_scale
  - [2] https://www.data.jma.go.jp/eqev/data/kyoshin/kaisetsu/calc_sindo.html
  - [3] http://www.daime.co.jp/gifujisin/data/skasokudo.html
  - [4] https://create.arduino.cc/projecthub/mircemk/sensitive-mpu6050-seismometer-with-data-logger-9e6bf5
  - [5] https://github.com/JonHub/Filters
  - [6] http://www.bousai.go.jp/jishin/syuto/denkikasaitaisaku/2/pdf/siryou3.pdf
  - [7] https://github.com/emilv/ArduinoSort
  - [8] https://www.bcj.or.jp/download/wave/
  
### Filters library
  - [https://github.com/JonHub/Filters]https://github.com/JonHub/Filters
### BMP388 library
  - [https://github.com/adafruit/Adafruit_BMP3XX]https://github.com/adafruit/Adafruit_BMP3XX
### PubSubClient
  - [https://github.com/knolleary/pubsubclient](https://github.com/knolleary/pubsubclient)
### MQTT server with visiualization tools
  - [https://thingsboard.io/] (https://thingsboard.io/)

## License
* Copyright 2022 by coniferconifer
* Apache License

*/

//BMP388 balometric pressure sensor from Adafruit
#define PRESS_OFFSET 0.00 //BMP388 has precise ablosule balometric pressure , need to be changed on your device 
#define TEMP_OFFSET 3.56 // temperature offset in degree
boolean BMP388exists=false;
#define SEISMIC_OBSERVATION_PERIOD 1000*30 //30sec
#define SAMPLING_SPEED 10 //10Hz
#define LAST_BUFFERSIZE 30 // find the acceleration in the last 3sec , find af that continues just 0.3sec ,
//                            assuming loopforSeismo() loops at 100msec interval
//#define SIMULATION
//#define BCJL1 //use BCJ-L1 wave , else BCJ-L2 is selected
#ifdef SIMULATION
#define BCJL1_SIZE 6000
#define BCJL2_SIZE 12000
//  bcj-simulation.h includes
//  float bcjL1_wave[BCJL1_SIZE]; //10msec sampling 60sec
//  float bcjL2_wave[BCJL2_SIZE];//10msec sampling 120sec
#include "bcj-simulation.h"
#endif

#define BTSERIAL
#ifdef BTSERIAL
const char *BTpin = "0000";
#endif
#ifdef SIMULATION
#undef BTSERIAL //avoid BTSERIAL use in case of simulation 
#endif

#ifdef BTSERIAL
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
#endif

//#define VERBOSE

#define MAX1G 16384.0 // 1G in 2Gmax mode
#define GRAVITY 980.0 // 1G in cm/sec^2

#include "Wire.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
Adafruit_BMP3XX bmp;

//FreeRTOS
boolean publishGo = false;
QueueHandle_t xQueue;
TaskHandle_t Task1, Task2, Task3;
volatile SemaphoreHandle_t semaI2C;
#define  WAIT_FOR_SEMAPHORE_TIME  (100UL)

//MPU6050 paramter
const int MPU_addr = 0x68; // I2C address of the MPU-6050

#include <Filters.h> //https://github.com/JonHub/Filters 
FilterOnePole XFHigh(HIGHPASS, 0.2), YFHigh(HIGHPASS, 0.2), ZFHigh(HIGHPASS, 0.2);//0.2Hz high pass filter
//FilterOnePole XFHigh(HIGHPASS, 0.625), YFHigh(HIGHPASS, 0.625), ZFHigh(HIGHPASS, 0.625);//0.625Hz high pass filter

void codeForSeismo(void * parameter);
// supports multi WiFi Access Points
#include <WiFi.h>
#include "credentials.h"
#define MAX_TRY 15
char* ssidArray[] = { WIFI_SSID , WIFI_SSID1, WIFI_SSID2};
char* passwordArray[] = {WIFI_PASS, WIFI_PASS1, WIFI_PASS2};
#include <PubSubClient.h>
WiFiClient wifiClient;

#define MQTTPORT 2883 

char* tokenArray[] = { TOKEN , TOKEN1, TOKEN2};
char* serverArray[] = {SERVER, SERVER1, SERVER2};
PubSubClient client(serverArray[0], MQTTPORT, wifiClient);
#define MQTTRETRY 1
#define DEVICE_TYPE "Seismo" // 
String clientId = DEVICE_TYPE ; //uniq clientID will be generated from MAC
char topic[] = "v1/devices/me/telemetry"; //for thingsboard.org
// index of WiFi access point
int AP = -1; // access point is not yet found , -1 means not WiFi , but Simple BLE mode
boolean mqttflag = false;

#define MPU6050_AUX_VDDIO          0x01   // R/W
#define MPU6050_SMPLRT_DIV         0x19   // R/W
#define MPU6050_CONFIG             0x1A   // R/W
#define MPU6050_GYRO_CONFIG        0x1B   // R/W
#define MPU6050_ACCEL_CONFIG       0x1C   // R/W
#define MPU6050_FF_THR             0x1D   // R/W
#define MPU6050_FF_DUR             0x1E   // R/W
#define MPU6050_MOT_THR            0x1F   // R/W
#define MPU6050_MOT_DUR            0x20   // R/W
#define MPU6050_ZRMOT_THR          0x21   // R/W
#define MPU6050_ZRMOT_DUR          0x22   // R/W
#define MPU6050_FIFO_EN            0x23   // R/W
#define MPU6050_I2C_MST_CTRL       0x24   // R/W
#define MPU6050_I2C_SLV0_ADDR      0x25   // R/W
#define MPU6050_I2C_SLV0_REG       0x26   // R/W
#define MPU6050_I2C_SLV0_CTRL      0x27   // R/W
#define MPU6050_I2C_SLV1_ADDR      0x28   // R/W
#define MPU6050_I2C_SLV1_REG       0x29   // R/W
#define MPU6050_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU6050_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU6050_I2C_SLV2_REG       0x2C   // R/W
#define MPU6050_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU6050_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU6050_I2C_SLV3_REG       0x2F   // R/W
#define MPU6050_I2C_SLV3_CTRL      0x30   // R/W
#define MPU6050_I2C_SLV4_ADDR      0x31   // R/W
#define MPU6050_I2C_SLV4_REG       0x32   // R/W
#define MPU6050_I2C_SLV4_DO        0x33   // R/W
#define MPU6050_I2C_SLV4_CTRL      0x34   // R/W
#define MPU6050_I2C_SLV4_DI        0x35   // R  
#define MPU6050_I2C_MST_STATUS     0x36   // R
#define MPU6050_INT_PIN_CFG        0x37   // R/W
#define MPU6050_INT_ENABLE         0x38   // R/W
#define MPU6050_INT_STATUS         0x3A   // R  
#define MPU6050_ACCEL_XOUT_H       0x3B   // R  
#define MPU6050_ACCEL_XOUT_L       0x3C   // R  
#define MPU6050_ACCEL_YOUT_H       0x3D   // R  
#define MPU6050_ACCEL_YOUT_L       0x3E   // R  
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R  
#define MPU6050_ACCEL_ZOUT_L       0x40   // R  
#define MPU6050_TEMP_OUT_H         0x41   // R  
#define MPU6050_TEMP_OUT_L         0x42   // R  
#define MPU6050_GYRO_XOUT_H        0x43   // R  
#define MPU6050_GYRO_XOUT_L        0x44   // R  
#define MPU6050_GYRO_YOUT_H        0x45   // R  
#define MPU6050_GYRO_YOUT_L        0x46   // R  
#define MPU6050_GYRO_ZOUT_H        0x47   // R  
#define MPU6050_GYRO_ZOUT_L        0x48   // R  
#define MPU6050_EXT_SENS_DATA_00   0x49   // R  
#define MPU6050_EXT_SENS_DATA_01   0x4A   // R  
#define MPU6050_EXT_SENS_DATA_02   0x4B   // R  
#define MPU6050_EXT_SENS_DATA_03   0x4C   // R  
#define MPU6050_EXT_SENS_DATA_04   0x4D   // R  
#define MPU6050_EXT_SENS_DATA_05   0x4E   // R  
#define MPU6050_EXT_SENS_DATA_06   0x4F   // R  
#define MPU6050_EXT_SENS_DATA_07   0x50   // R  
#define MPU6050_EXT_SENS_DATA_08   0x51   // R  
#define MPU6050_EXT_SENS_DATA_09   0x52   // R  
#define MPU6050_EXT_SENS_DATA_10   0x53   // R  
#define MPU6050_EXT_SENS_DATA_11   0x54   // R  
#define MPU6050_EXT_SENS_DATA_12   0x55   // R  
#define MPU6050_EXT_SENS_DATA_13   0x56   // R  
#define MPU6050_EXT_SENS_DATA_14   0x57   // R  
#define MPU6050_EXT_SENS_DATA_15   0x58   // R  
#define MPU6050_EXT_SENS_DATA_16   0x59   // R  
#define MPU6050_EXT_SENS_DATA_17   0x5A   // R  
#define MPU6050_EXT_SENS_DATA_18   0x5B   // R  
#define MPU6050_EXT_SENS_DATA_19   0x5C   // R  
#define MPU6050_EXT_SENS_DATA_20   0x5D   // R  
#define MPU6050_EXT_SENS_DATA_21   0x5E   // R  
#define MPU6050_EXT_SENS_DATA_22   0x5F   // R  
#define MPU6050_EXT_SENS_DATA_23   0x60   // R  
#define MPU6050_MOT_DETECT_STATUS  0x61   // R  
#define MPU6050_I2C_SLV0_DO        0x63   // R/W
#define MPU6050_I2C_SLV1_DO        0x64   // R/W
#define MPU6050_I2C_SLV2_DO        0x65   // R/W
#define MPU6050_I2C_SLV3_DO        0x66   // R/W
#define MPU6050_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU6050_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU6050_MOT_DETECT_CTRL    0x69   // R/W
#define MPU6050_USER_CTRL          0x6A   // R/W
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_PWR_MGMT_2         0x6C   // R/W
#define MPU6050_FIFO_COUNTH        0x72   // R/W
#define MPU6050_FIFO_COUNTL        0x73   // R/W
#define MPU6050_FIFO_R_W           0x74   // R/W
#define MPU6050_WHO_AM_I           0x75   // R

#define MPU6050_D0 0
#define MPU6050_D1 1
#define MPU6050_D2 2
#define MPU6050_D3 3
#define MPU6050_D4 4
#define MPU6050_D5 5
#define MPU6050_D6 6
#define MPU6050_D7 7

#define MPU6050_DLPF_CFG0     MPU6050_D0
#define MPU6050_DLPF_CFG1     MPU6050_D1
#define MPU6050_DLPF_CFG2     MPU6050_D2
#define MPU6050_EXT_SYNC_SET0 MPU6050_D3
#define MPU6050_EXT_SYNC_SET1 MPU6050_D4
#define MPU6050_EXT_SYNC_SET2 MPU6050_D5

#define MPU6050_EXT_SYNC_SET_0 (0)
#define MPU6050_EXT_SYNC_SET_1 (bit(MPU6050_EXT_SYNC_SET0))
#define MPU6050_EXT_SYNC_SET_2 (bit(MPU6050_EXT_SYNC_SET1))
#define MPU6050_EXT_SYNC_SET_3 (bit(MPU6050_EXT_SYNC_SET1)|bit(MPU6050_EXT_SYNC_SET0))
#define MPU6050_EXT_SYNC_SET_4 (bit(MPU6050_EXT_SYNC_SET2))
#define MPU6050_EXT_SYNC_SET_5 (bit(MPU6050_EXT_SYNC_SET2)|bit(MPU6050_EXT_SYNC_SET0))
#define MPU6050_EXT_SYNC_SET_6 (bit(MPU6050_EXT_SYNC_SET2)|bit(MPU6050_EXT_SYNC_SET1))
#define MPU6050_EXT_SYNC_SET_7 (bit(MPU6050_EXT_SYNC_SET2)|bit(MPU6050_EXT_SYNC_SET1)|bit(MPU6050_EXT_SYNC_SET0))

#define MPU6050_EXT_SYNC_DISABLED     MPU6050_EXT_SYNC_SET_0
#define MPU6050_EXT_SYNC_TEMP_OUT_L   MPU6050_EXT_SYNC_SET_1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L  MPU6050_EXT_SYNC_SET_2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L  MPU6050_EXT_SYNC_SET_3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L  MPU6050_EXT_SYNC_SET_4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L MPU6050_EXT_SYNC_SET_5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L MPU6050_EXT_SYNC_SET_6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L MPU6050_EXT_SYNC_SET_7

#define MPU6050_DLPF_CFG_0 (0)
#define MPU6050_DLPF_CFG_1 (bit(MPU6050_DLPF_CFG0))
#define MPU6050_DLPF_CFG_2 (bit(MPU6050_DLPF_CFG1))
#define MPU6050_DLPF_CFG_3 (bit(MPU6050_DLPF_CFG1)|bit(MPU6050_DLPF_CFG0))
#define MPU6050_DLPF_CFG_4 (bit(MPU6050_DLPF_CFG2))
#define MPU6050_DLPF_CFG_5 (bit(MPU6050_DLPF_CFG2)|bit(MPU6050_DLPF_CFG0))
#define MPU6050_DLPF_CFG_6 (bit(MPU6050_DLPF_CFG2)|bit(MPU6050_DLPF_CFG1))
#define MPU6050_DLPF_CFG_7 (bit(MPU6050_DLPF_CFG2)|bit(MPU6050_DLPF_CFG1)|bit(MPU6050_DLPF_CFG0))

#define MPU6050_DLPF_260HZ    MPU6050_DLPF_CFG_0
#define MPU6050_DLPF_184HZ    MPU6050_DLPF_CFG_1
#define MPU6050_DLPF_94HZ     MPU6050_DLPF_CFG_2
#define MPU6050_DLPF_44HZ     MPU6050_DLPF_CFG_3
#define MPU6050_DLPF_21HZ     MPU6050_DLPF_CFG_4
#define MPU6050_DLPF_10HZ     MPU6050_DLPF_CFG_5
#define MPU6050_DLPF_5HZ      MPU6050_DLPF_CFG_6
#define MPU6050_DLPF_RESERVED MPU6050_DLPF_CFG_7

#define MPU6050_ACCEL_HPF0 MPU6050_D0
#define MPU6050_ACCEL_HPF1 MPU6050_D1
#define MPU6050_ACCEL_HPF2 MPU6050_D2
#define MPU6050_AFS_SEL0   MPU6050_D3
#define MPU6050_AFS_SEL1   MPU6050_D4
#define MPU6050_ZA_ST      MPU6050_D5
#define MPU6050_YA_ST      MPU6050_D6
#define MPU6050_XA_ST      MPU6050_D7

#define MPU6050_ACCEL_HPF_0 (0)
#define MPU6050_ACCEL_HPF_1 (bit(MPU6050_ACCEL_HPF0))
#define MPU6050_ACCEL_HPF_2 (bit(MPU6050_ACCEL_HPF1))
#define MPU6050_ACCEL_HPF_3 (bit(MPU6050_ACCEL_HPF1)|bit(MPU6050_ACCEL_HPF0))
#define MPU6050_ACCEL_HPF_4 (bit(MPU6050_ACCEL_HPF2))
#define MPU6050_ACCEL_HPF_7 (bit(MPU6050_ACCEL_HPF2)|bit(MPU6050_ACCEL_HPF1)|bit(MPU6050_ACCEL_HPF0))

#define MPU6050_ACCEL_HPF_RESET  MPU6050_ACCEL_HPF_0
#define MPU6050_ACCEL_HPF_5HZ    MPU6050_ACCEL_HPF_1
#define MPU6050_ACCEL_HPF_2_5HZ  MPU6050_ACCEL_HPF_2
#define MPU6050_ACCEL_HPF_1_25HZ MPU6050_ACCEL_HPF_3
#define MPU6050_ACCEL_HPF_0_63HZ MPU6050_ACCEL_HPF_4
#define MPU6050_ACCEL_HPF_HOLD   MPU6050_ACCEL_HPF_7

#define MPU6050_AFS_SEL_0 (0)
#define MPU6050_AFS_SEL_1 (bit(MPU6050_AFS_SEL0))
#define MPU6050_AFS_SEL_2 (bit(MPU6050_AFS_SEL1))
#define MPU6050_AFS_SEL_3 (bit(MPU6050_AFS_SEL1)|bit(MPU6050_AFS_SEL0))

#define MPU6050_AFS_SEL_2G  MPU6050_AFS_SEL_0
#define MPU6050_AFS_SEL_4G  MPU6050_AFS_SEL_1
#define MPU6050_AFS_SEL_8G  MPU6050_AFS_SEL_2
#define MPU6050_AFS_SEL_16G MPU6050_AFS_SEL_3




int initWiFi() {
  int i ;
  int numaccesspt = (sizeof(ssidArray) / sizeof((ssidArray)[0]));

#ifdef VERBOSE
  Serial.print("Number of Access Point = "); Serial.println(numaccesspt);
#endif
  for (i = 0;  i < numaccesspt; i++) {
#ifdef VERBOSE
    Serial.print("WiFi connecting to "); Serial.println(ssidArray[i]);
#endif
    WiFi.mode(WIFI_OFF);
    WiFi.begin(ssidArray[i], passwordArray[i]);

    int j;
    for (j = 0; j < MAX_TRY; j++) {
      if (WiFi.status() == WL_CONNECTED) {

        int rssi = WiFi.RSSI();
        Serial.printf("RSSI= %d\n", rssi);
        //        displayWiFiAP(i,  rssi);
        //        delay(3000);
        //configTime(TIMEZONE * 3600L, 0,  NTP1, NTP2, NTP3);
#ifdef VERBOSE
        Serial.print("WiFi connected as IP address: "); Serial.println(WiFi.localIP());
#endif
        return (i);
      }
      delay(500);
#ifdef VERBOSE
      Serial.print(".");
#endif

    }
#ifdef VERBOSE
    Serial.println(" can not connect to WiFi AP");
#endif

  }
  return (-1);
}
void setupWiFi() {
  // generate uniq clientId
  uint64_t chipid;
  chipid = ESP.getEfuseMac();
  clientId += "-";
  clientId += String((uint32_t)chipid, HEX);
  Serial.println("clientId :" + clientId);

  AP = initWiFi();
  Serial.printf("WiFi AP=%d\r\n", AP);
  if ( AP != -1) {  // found  WiFi AP
    client.setClient(wifiClient);
    client.setServer(serverArray[AP], MQTTPORT); // MQTT server for NodeRED or MQTT by Thingsboarxd
    digitalWrite(GPIO_NUM_2, HIGH);
    delay(3000);
  } else {
    delay(3000);
    AP = initWiFi();
  }
}
int initWiFi_retry() {
#ifdef VERBOSE
  Serial.print("initWiFi_retry() WiFi connecting to "); Serial.println(ssidArray[AP]);
#endif
  //  Serial.print(" "); Serial.print(passwordArray[AP]);
  WiFi.mode(WIFI_OFF);
  WiFi.begin(ssidArray[AP], passwordArray[AP]);

  int j;
  for (j = 0; j < MAX_TRY; j++) {
    if (WiFi.status() == WL_CONNECTED) {
#ifdef VERBOSE
      Serial.print("WiFi connected, IP address: "); Serial.println(WiFi.localIP());
#endif
      //configTime(TIMEZONE * 3600L, 0,  NTP1, NTP2, NTP3);

      return (AP);
    }
    delay(500);
#ifdef VERBOSE
    Serial.print(".");
#endif
  }
#ifdef VERBOSE
  Serial.println(" can not connect to WiFi AP");
#endif

  return (-1);
}

void publishToMQTT(float press, float IfF, float IfF_h, float temperature) {

  String payload = "{";
  if (BMP388exists==true){
    payload += "\"press1\":"; payload += press - PRESS_OFFSET ; payload += ",";
    payload += "\"temp1\":"; payload += temperature - TEMP_OFFSET  ; payload += ","; 
  }
  payload += "\"IfF\":"; payload += IfF ; payload += ",";
  payload += "\"IfF_h\":"; payload += IfF_h;
  payload += "}";
  // dont send too long string to MQTT server
  // max 128byte

#ifdef VERBOSE
  Serial.print("Sending payload: "); Serial.println(payload);
  Serial.print("AP = "); Serial.println(AP);
  Serial.print("Reconnecting client to "); Serial.println(serverArray[AP]);
#endif
  int mqttloop = 0;
  while (1) { // for thingsboard MQTT server
    mqttflag = client.connect(clientId.c_str(),  tokenArray[AP], NULL);

    if (mqttflag == true) break;
    Serial.print("-"); delay(500);
    mqttloop++;
    if (mqttloop > MQTTRETRY) { //there may be something wrong
      mqttflag = false;
      initWiFi_retry();
      // ESP.restart();
      break;
    }
  }

  if (mqttflag == true) {
    if (client.publish(topic, (char*) payload.c_str())) {
#ifdef VERBOSE
      Serial.println("Publish ok");
#endif
    } else {
#ifdef VERBOSE
      Serial.println("Publish failed");
#endif
    }

  } else {
#ifdef VERBOSE
    Serial.println("unable to connect to MQTT server");
#endif
  }

}

extern float press, IfF, IfFmax, If_h , IfF_hmax,  temperature;
// when publishGo is true via xQueue
// codeForMQTT will publish balometric pressure , Seismic index IfF and IfF_h , temperature
void codeForMQTT(void * parameter)
{
  portTickType xLastWakeTime;
  BaseType_t result;
  boolean publishGo = false;
  while (1) {
    if ( xQueue != 0 ) {
      result = xQueueReceive(xQueue, &publishGo, portMAX_DELAY);
      if ( result != pdPASS)
      {
        Serial.printf("queue receive failed\r\n");
      } else {
        if ( publishGo == true) {
          publishGo=false;
          publishToMQTT( press, IfFmax , IfF_hmax, temperature);
        }
      }
    }
   
  }
}
float press = 1000.0; //observed balometric pressure filtered by low pass filter
float temperature = 20.0;
int initBMP388() {
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    return -1;
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  if (! bmp.performReading()) {
    Serial.println("BMP388: Failed to perform reading :(");
    return -1;
  }

  int i;
  for (i = 0; i < 10; i++) {
    bmp.performReading();
    press = bmp.pressure / 100.0;
    temperature = bmp.temperature;
    delay(100);
  }
  return 0;
}
extern int initMPU6050();
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("ESP32-seismometer.ino");
#ifdef BTSERIAL
  Serial.println("Setup Android terminal to pair with Seisemo-BT-server");
  SerialBT.setPin(BTpin);
  SerialBT.begin("Seismo-BT-server");
  Serial.println("The device started, now you can pair it with bluetooth!");
#endif
  pinMode(GPIO_NUM_2, OUTPUT); //LED

  Wire.begin(GPIO_NUM_21, GPIO_NUM_22);
//  Wire.setClock(400000L);
  //http://www.azusa-st.com/kjm/FreeRtos/API/semaphores/vSemaphoreTake.html
  //I2C bus is shared by two devices , semaI2C takes exclusive I2C bus control
  semaI2C = xSemaphoreCreateMutex();
  xSemaphoreGive( semaI2C );  /* give one semaphore */
  if (initBMP388()!=0){
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  } else {
    Serial.println("BMP388 found");
    BMP388exists=true;
  }
  setupWiFi();
  delay(3000); //wait mechanical stability of MPU6050 after you turn on  ESP32 board
  if (initMPU6050()==-1) {
    Serial.printf("MPU6050 not detected\r\n");
    while(1);
  }
  clearLastBuffer();//LastBuffer is used to find acceleration exceeds 0.3sec
  // publishGo is used to signal MQTT publish task to go
  xQueue = xQueueCreate(1, sizeof(publishGo));
  if (xQueue == NULL)
  {
    Serial.printf("queue creation failed \r\n");
    while (1);
  } else {
    if ( BMP388exists==true){
      xTaskCreatePinnedToCore( codeForPress, "codeForPress", 4000, NULL, 1, &Task1, 1); //core 1
    }
    xTaskCreatePinnedToCore( codeForSeismo, "codeForSeismo", 4000, NULL, 1, &Task2, 1); //core 1
    delay(3000);//publish after sensor stabilization time
    xTaskCreatePinnedToCore( codeForMQTT, "codeForMQTT", 4000, NULL, 1, &Task3, 1); //core 1
  }
}

void loop() {
}
void codeForPress(void * parameter) {
  float r = 0.9;
  while (1) {
    if (xSemaphoreTake( semaI2C, pdMS_TO_TICKS( WAIT_FOR_SEMAPHORE_TIME )  ) == pdTRUE ) {
      if (!bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
      } else {
        float readPress = bmp.pressure / 100.0;
        float readTemp = bmp.temperature;
        press = press * r + readPress * (1.0 - r);
        temperature = temperature * r + readTemp * (1.0 - r);
        //Serial.printf("temp %f %f \r\n", readTemp, temperature);
        //   Serial.printf("press %f %f \r\n", (readPress - 1012.2) * 10.0, (press - 1012.2) * 10.0);
      }
      xSemaphoreGive( semaI2C );  /* give one semaphore */
    }
    delay(100);
  }
}

int initMPU6050() {
  unsigned char c;
  int error;
  Wire.beginTransmission(MPU_addr);
  Wire.write(MPU6050_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 1, true);
  c = Wire.read();
  Serial.print("MPU9250 / MPU6050 WHO_AM_I (0x68 / 0x71): ");7
  Serial.println(c, HEX);
  if (!(c==0x71 || c==0x68)) {
    return -1;
  }
  Wire.beginTransmission(MPU_addr);
  Wire.write(MPU6050_CONFIG);
  Wire.write(MPU6050_EXT_SYNC_DISABLED);
  Wire.write(MPU6050_ACCEL_CONFIG);
  Wire.write(MPU6050_AFS_SEL_2G);
  Wire.write(MPU6050_PWR_MGMT_1);  // PWR_MGMT_1 register
  Wire.write(0);                   // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
#ifdef SIMULATION
#else
  getOffsetMPU6050();
#endif
  return 0; //OK
}

void getAxAyAz( int16_t *AcX, int16_t *AcY , int16_t *AcZ) {
  if (xSemaphoreTake( semaI2C, pdMS_TO_TICKS( WAIT_FOR_SEMAPHORE_TIME )  ) == pdTRUE ) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
    *AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    *AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    *AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    xSemaphoreGive( semaI2C );  /* give one semaphore */
  }
}
int16_t offsetX, offsetY, offsetZ; //MPU9050 offset , placed horizontaly
void getOffsetMPU6050() {
  int i;
  int16_t AcX, AcY, AcZ;
  int32_t tempAcX = 0;
  int32_t tempAcY = 0;
  int32_t tempAcZ = 0;
#define CALIBCNT 1000
  for (i = 0; i < CALIBCNT; i++) {
    getAxAyAz( &AcX, &AcY , &AcZ);
    // Serial.printf("inner %d , %d , %d \r\n",AcX,AcY,AcZ-16384);
    tempAcX = tempAcX + AcX;
    tempAcY = tempAcY + AcY;
    tempAcZ = tempAcZ + AcZ;
    delay(10);
    if (i % 100 == 0) Serial.print(".");
  }
  offsetX = tempAcX / CALIBCNT;
  offsetY = tempAcY / CALIBCNT;
  offsetZ = tempAcZ / CALIBCNT;

  float afinG = sqrt((float)offsetX * (float)offsetX + (float)offsetY * (float)offsetY + (float)offsetZ * (float)offsetZ);

  Serial.printf("\r\nMPU6050 offset %d %d %d  total cm/sec^2 %f\r\n", offsetX, offsetY, offsetZ , afinG / MAX1G * GRAVITY);
  delay(3000);//wait for mechanical stabilization
}
//shared variables
float IfF = 0.0;  //seismic index
float IfF_h = 0.0; //seismic index for horizontal acceleration
float IfFtemp = 0.0;
float IfFmax = 0.0; //max IfF during observation period
float IfF_htemp = 0.0;
float IfF_hmax = 0.0;//max IfF_h during observation period


void codeForSeismo( void * parameter)
{
  BaseType_t result;
  int i = 0;
  long ts;
  long ts_now;
  boolean publishGo;

  while (1) {
    if (i == 0)  {
      ts = millis();
    }
    loopforSeismo(); //update IfF IfF_h every 100msec
    //hold max IfF,IfF_h
    if ( IfF > IfFtemp )
    {
      IfFtemp = IfF;
    }
    if ( IfF_h > IfF_htemp ) {
      IfF_htemp = IfF_h;
    }
    i++;
    ts_now = millis();
    if ( ts_now - ts > SEISMIC_OBSERVATION_PERIOD ) { //get max IfF during observation period
      IfFmax = IfFtemp; IfFtemp = 0.0;
      IfF_hmax = IfF_htemp; IfF_htemp = 0.0;
      i = 0;

      publishGo = true;
      result = xQueueSend(xQueue, &publishGo, 10);
#ifdef VERBOSE
      Serial.printf("publishGo: %d\r\n",ts_now);
#endif
      if (result != pdPASS)
      {
        Serial.printf("xQueue failed\r\n");
      }
    }
  }

}
// https://github.com/emilv/ArduinoSort
#include <ArduinoSort.h>
long loopCnt = 0;
long bufferCnt = 0;

float IfF_h_last[LAST_BUFFERSIZE];
float IfF_last[LAST_BUFFERSIZE];
float temp_last[LAST_BUFFERSIZE];

void clearLastBuffer() {
  int i;
  for (i = 0; i < LAST_BUFFERSIZE; i++) {
    IfF_h_last[i] = 0.0;
    IfF_last[i] = 0.0;
  }
}
void loopforSeismo() {
  int16_t AcX = 0, AcY = 0, AcZ = 0;
  int32_t tempAcX = 0, tempAcY = 0, tempAcZ = 0;

  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();// Initialise the xLastWakeTime variable with the current time.
  const portTickType xFrequency = 100 ; //100msec loop
#define LOOPNUM 10

#ifdef SIMULATION
  extern float bcjL1_wave[];
  extern float bcjL2_wave[];

  for (int i = 0; i < LOOPNUM; i++) {
#ifdef BCJL1
    AcX = (int16_t)(bcjL1_wave[loopCnt % BCJL1_SIZE] / GRAVITY * MAX1G);
#else
    AcX = (int16_t)(bcjL2_wave[loopCnt % BCJL2_SIZE] / GRAVITY * MAX1G);
#endif
    AcY = AcX;
    AcZ = AcX / 2; //http://www.bousai.go.jp/jishin/syuto/denkikasaitaisaku/2/pdf/siryou3.pdf
    tempAcX = tempAcX + AcX;
    tempAcY = tempAcY + AcY;
    tempAcZ = tempAcZ + AcZ;
    if ( i < LOOPNUM - 1) { //skip delay at last loop
      delay(1000 / (SAMPLING_SPEED * LOOPNUM));
    }
    loopCnt++;
  }
  // get averaged Ac?
  AcX = tempAcX / LOOPNUM;
  AcY = tempAcY / LOOPNUM;
  AcZ = tempAcZ / LOOPNUM;
#else
  for (int i = 0; i < LOOPNUM; i++) {
    getAxAyAz( &AcX, &AcY , &AcZ);
    // Serial.printf("inner %d , %d , %d \r\n",AcX,AcY,AcZ-16384);
    tempAcX = tempAcX + AcX;
    tempAcY = tempAcY + AcY;
    tempAcZ = tempAcZ + AcZ;
    if ( i < (LOOPNUM - 1)) { //skip delay at last loop
      delay(1000 / (SAMPLING_SPEED * LOOPNUM));
    }
  }
  //get avearge
  AcX = tempAcX / LOOPNUM;
  AcY = tempAcY / LOOPNUM;
  AcZ = tempAcZ / LOOPNUM;

  AcX = AcX - offsetX;
  AcY = AcY - offsetY;
  AcZ = AcZ - offsetZ;
#endif

  // convert in G=980gal 1G=32768/2
  XFHigh.input(AcX / MAX1G);
  YFHigh.input(AcY / MAX1G);
  ZFHigh.input(AcZ / MAX1G);

#define REMOVE_GRAVITY
#ifdef REMOVE_GRAVITY
  //  Serial.printf("%f %f %f \r\n",XFHigh.output(),YFHigh.output(),ZFHigh.output());
  float af_h = GRAVITY * sqrt(XFHigh.output() * XFHigh.output() + YFHigh.output() * YFHigh.output() ); //in gal
  float af = GRAVITY * sqrt(XFHigh.output() * XFHigh.output() + YFHigh.output() * YFHigh.output() + ZFHigh.output() * ZFHigh.output() ); //in gal
#else
  // without soft high pass filter 1Hz
  float af_h = GRAVITY * sqrt( (float)AcX * (float)AcX / MAX1G / MAX1G + (float)AcY * (float)AcY / MAX1G / MAX1G  ); //in gal
  float af = GRAVITY * sqrt( (float)AcX * (float)AcX / MAX1G / MAX1G + (float)AcY * (float)AcY / MAX1G / MAX1G + (float)AcZ * (float)AcZ / MAX1G / MAX1G ); //in gal
  //  float af_h = GRAVITY * sqrt( (float)AcX*(float)AcX/MAX1G/MAX1G + (float)AcY*(float)AcY/MAX1G/MAX1G );//in gal
#endif


  float If = 2.0 * log10( af ) + 0.94; //https://www.jma.go.jp/jma/press/0903/30a/jma-shindo-kaisetsu-pub.pdf
  float If_horizontal = 2.0 * log10( af_h ) + 0.94;
  // af in cm/sec2
  // Serial.printf("%f ,%f\r\n",af,If);
  if (If < 0.0 ) If = 0.0;
  if (If_horizontal < 0.0 ) If_horizontal = 0.0;

  IfF_last[bufferCnt] = If;
  IfF_h_last[bufferCnt] = If_horizontal;

  //find minimum in the last 3 observation , correspondig to 0.3sec
  //  findMin( IfF_last , IfF_h_last, 3 , &IfF , &IfF_h);
  for (int j = 0; j < LAST_BUFFERSIZE; j++) {
    temp_last[j] = IfF_last[j];
  }
//#define DEBUG
#ifdef DEBUG
  Serial.print("If raw: ");
  for (int j = 0; j < LAST_BUFFERSIZE; j++) {
    Serial.print(temp_last[j]); Serial.print(",");
  }
  Serial.println();

#endif
  sortArrayReverse(temp_last, LAST_BUFFERSIZE);

#ifdef DEBUG
  Serial.print("sorted ");
  for (int j = 0; j < LAST_BUFFERSIZE; j++) {
    Serial.print(temp_last[j]); Serial.print(",");
  }
  Serial.println();
#endif

  IfF = temp_last[2]; // acceleration at least 0.3sec

  for (int j = 0; j < LAST_BUFFERSIZE; j++) {
    temp_last[j] = IfF_h_last[j];
  }
  sortArrayReverse(temp_last, LAST_BUFFERSIZE);
  IfF_h = temp_last[2];

  bufferCnt++;
  if (bufferCnt > (LAST_BUFFERSIZE - 1)) bufferCnt = 0;

  //Serial.printf("ts If IfF Iftemp Ifmax %d %f %f %f %f\r\n", millis(), If, IfF, IfFtemp, IfFmax);
  //Serial.printf("af_h If  IfF Iftemp Ifmax  %f %f %f %f %f\r\n", af_h , If, IfF, IfFtemp, IfFmax);
  Serial.println("If, IfF, Iftemp, Ifmax");
  Serial.printf("%f , %f , %f ,%f \r\n", If, IfF, IfFtemp, IfFmax);
#ifdef BTSERIAL
    SerialBT.printf("%3.0f , %3.0f , %1.2f , %1.2f , %1.2f\r\n", af, af_h, If,IfF, IfFmax);
#endif
    vTaskDelayUntil( &xLastWakeTime, xFrequency ); //wait until 100msec for this loop
}
