#include <SparkFunBME280.h>
BME280 mySensor; // start BME280

#include "quaternionFilters.h"
#include "MPU9250.h"
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0 
MPU9250 myIMU; // start MPU9250

#include <HMC5883L.h>
HMC5883L compass;


#include "SSD1306Wire.h" 
#include "OLEDDisplayUi.h"
#include "images.h"
SSD1306Wire  display(0x3c, 4, 5);
OLEDDisplayUi ui     ( &display );

FSInfo fs_info;
uint32_t realSize = ESP.getFlashChipRealSize();
float fileTotalKB = (float)fs_info.totalBytes / 1024.0; 
float fileUsedKB = (float)fs_info.usedBytes / 1024.0; 
float fsFree = fileTotalKB - fileUsedKB;

float flashChipSize = (float)ESP.getFlashChipSize() / 1024.0 / 1024.0;
float realFlashChipSize = (float)ESP.getFlashChipRealSize() / 1024.0 / 1024.0;
float flashFreq = (float)ESP.getFlashChipSpeed() / 1000.0 / 1000.0;
FlashMode_t ideMode = ESP.getFlashChipMode();

String millis2time() {
  String Time = "";
  unsigned long ss;
  byte mm, hh;
  ss = millis() / 1000;
  hh = ss / 3600;
  mm = (ss - hh * 3600) / 60;
  ss = (ss - hh * 3600) - mm * 60;
  if (hh < 10)Time += "0";
  Time += (String)hh + ":";
  if (mm < 10)Time += "0";
  Time += (String)mm + ":";
  if (ss < 10)Time += "0";
  Time += (String)ss;
  return String(Time);
}

// sensor functions...
// BME280
void setupBME280() {  
  DBG_OUTPUT_PORT.printf("\nSetting up BME280....\n");
  mySensor.settings.commInterface = I2C_MODE;
  mySensor.settings.I2CAddress = 0x76;  //bme280 address
  mySensor.settings.runMode = 3; //0, Sleep mode, 1 or 2, Forced mode, 3, Normal mode
  mySensor.settings.tStandby = 0; //0, 0.5ms, 1, 62.5ms, 2, 125ms, 3, 250ms, 4, 500ms, 5, 1000ms, 6, 10ms, 7, 20ms
  mySensor.settings.filter = 0; // 0, filter off, 1,coefficients = 2, 2,coefficients = 4, 3,coefficients = 8, 4, coefficients = 16
  mySensor.settings.tempOverSample = 1; //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  mySensor.settings.pressOverSample = 1; //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  mySensor.settings.humidOverSample = 1; //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  delay(20);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  DBG_OUTPUT_PORT.print("Started BME280... result of .begin(): 0x");
  DBG_OUTPUT_PORT.println(mySensor.begin(), HEX); // start BME280
  DBG_OUTPUT_PORT.printf("\n");
}

// MPU9250
void setupMPU9250() {
  DBG_OUTPUT_PORT.printf("Setting up MPY9250....\n");
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  DBG_OUTPUT_PORT.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  DBG_OUTPUT_PORT.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    DBG_OUTPUT_PORT.printf("MPU9250 is online...\n");

    myIMU.MPU9250SelfTest(myIMU.selfTest);
    DBG_OUTPUT_PORT.print("x-axis self test: acceleration trim within : ");
    DBG_OUTPUT_PORT.print(myIMU.selfTest[0],1); Serial.printf("% of factory value\n");
    DBG_OUTPUT_PORT.print("y-axis self test: acceleration trim within : ");
    DBG_OUTPUT_PORT.print(myIMU.selfTest[1],1); Serial.printf("% of factory value\n");
    DBG_OUTPUT_PORT.print("z-axis self test: acceleration trim within : ");
    DBG_OUTPUT_PORT.print(myIMU.selfTest[2],1); Serial.printf("% of factory value\n");
    DBG_OUTPUT_PORT.print("x-axis self test: gyration trim within : ");
    DBG_OUTPUT_PORT.print(myIMU.selfTest[3],1); Serial.printf("% of factory value\n");
    DBG_OUTPUT_PORT.print("y-axis self test: gyration trim within : ");
    DBG_OUTPUT_PORT.print(myIMU.selfTest[4],1); Serial.printf("% of factory value\n");
    DBG_OUTPUT_PORT.print("z-axis self test: gyration trim within : ");
    DBG_OUTPUT_PORT.print(myIMU.selfTest[5],1); Serial.printf("% of factory value\n");

    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    
    myIMU.initMPU9250();
    DBG_OUTPUT_PORT.printf("MPU9250 initialized for active data mode....\n");

    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    DBG_OUTPUT_PORT.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    DBG_OUTPUT_PORT.print(" I should be "); Serial.println(0x48, HEX);

    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    DBG_OUTPUT_PORT.print("AK8963 initialized for active data mode....\n");

      DBG_OUTPUT_PORT.print("X-Axis sensitivity adjustment value ");
      DBG_OUTPUT_PORT.println(myIMU.factoryMagCalibration[0], 2);
      DBG_OUTPUT_PORT.print("Y-Axis sensitivity adjustment value ");
      DBG_OUTPUT_PORT.println(myIMU.factoryMagCalibration[1], 2);
      DBG_OUTPUT_PORT.print("Z-Axis sensitivity adjustment value ");
      DBG_OUTPUT_PORT.println(myIMU.factoryMagCalibration[2], 2);

  } // if (c == 0x71)
  else
  {
    DBG_OUTPUT_PORT.print("Could not connect to MPU9250: 0x");
    DBG_OUTPUT_PORT.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  } 
}

void doMPU9250() {
   myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    myIMU.magBias[0] = 0; //+470.;
    myIMU.magBias[1] = 0; //+120.;
    myIMU.magBias[2] = 0; //+125.;

    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.factoryMagCalibration[0] -
               myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.factoryMagCalibration[1] -
               myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.factoryMagCalibration[2] -
               myIMU.magBias[2];

  myIMU.updateTime();

  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
                 
  myIMU.delt_t = millis() - myIMU.count;

      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      myIMU.yaw   = 12.23; // Magnetic Declination 
      myIMU.roll  *= RAD_TO_DEG;

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;

      myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        
    } 


// HMC5883L
void checkSettings() {
  DBG_OUTPUT_PORT.print("Selected range: ");
  
  switch (compass.getRange())
  {
    case HMC5883L_RANGE_0_88GA: DBG_OUTPUT_PORT.println("0.88 Ga"); break;
    case HMC5883L_RANGE_1_3GA:  DBG_OUTPUT_PORT.println("1.3 Ga"); break;
    case HMC5883L_RANGE_1_9GA:  DBG_OUTPUT_PORT.println("1.9 Ga"); break;
    case HMC5883L_RANGE_2_5GA:  DBG_OUTPUT_PORT.println("2.5 Ga"); break;
    case HMC5883L_RANGE_4GA:    DBG_OUTPUT_PORT.println("4 Ga"); break;
    case HMC5883L_RANGE_4_7GA:  DBG_OUTPUT_PORT.println("4.7 Ga"); break;
    case HMC5883L_RANGE_5_6GA:  DBG_OUTPUT_PORT.println("5.6 Ga"); break;
    case HMC5883L_RANGE_8_1GA:  DBG_OUTPUT_PORT.println("8.1 Ga"); break;
    default: DBG_OUTPUT_PORT.println("Bad range!");
  }
  DBG_OUTPUT_PORT.print("Selected Measurement Mode: ");
  switch (compass.getMeasurementMode())
  {  
    case HMC5883L_IDLE: DBG_OUTPUT_PORT.println("Idle mode"); break;
    case HMC5883L_SINGLE:  DBG_OUTPUT_PORT.println("Single-Measurement"); break;
    case HMC5883L_CONTINOUS:  DBG_OUTPUT_PORT.println("Continuous-Measurement"); break;
    default: DBG_OUTPUT_PORT.println("Bad mode!");
  }
  DBG_OUTPUT_PORT.print("Selected Data Rate: ");
  switch (compass.getDataRate())
  {  
    case HMC5883L_DATARATE_0_75_HZ: DBG_OUTPUT_PORT.println("0.75 Hz"); break;
    case HMC5883L_DATARATE_1_5HZ:  DBG_OUTPUT_PORT.println("1.5 Hz"); break;
    case HMC5883L_DATARATE_3HZ:  DBG_OUTPUT_PORT.println("3 Hz"); break;
    case HMC5883L_DATARATE_7_5HZ: DBG_OUTPUT_PORT.println("7.5 Hz"); break;
    case HMC5883L_DATARATE_15HZ:  DBG_OUTPUT_PORT.println("15 Hz"); break;
    case HMC5883L_DATARATE_30HZ: DBG_OUTPUT_PORT.println("30 Hz"); break;
    case HMC5883L_DATARATE_75HZ:  DBG_OUTPUT_PORT.println("75 Hz"); break;
    default: DBG_OUTPUT_PORT.println("Bad data rate!");
  }
  DBG_OUTPUT_PORT.print("Selected number of samples: ");
  switch (compass.getSamples())
  {  
    case HMC5883L_SAMPLES_1: DBG_OUTPUT_PORT.println("1"); break;
    case HMC5883L_SAMPLES_2: DBG_OUTPUT_PORT.println("2"); break;
    case HMC5883L_SAMPLES_4: DBG_OUTPUT_PORT.println("4"); break;
    case HMC5883L_SAMPLES_8: DBG_OUTPUT_PORT.println("8"); break;
    default: DBG_OUTPUT_PORT.println("Bad number of samples!");
  }
}

void setupHMC5883L() {
  DBG_OUTPUT_PORT.println("\nInitialize HMC5883L");
  while (!compass.begin())
  {
    DBG_OUTPUT_PORT.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_15HZ);
  compass.setSamples(HMC5883L_SAMPLES_1);
  // Check settings
  checkSettings();
}

// SSD1306
int screenW = 128;
int screenH = 64;
int clockCenterX = screenW/2;
int clockCenterY = ((screenH+16)/2)-16;   // top yellow part is 16 px height
int clockRadius = 23;

String twoDigits(int digits){
  if(digits < 10) {
    String i = '0'+String(digits);
    return i;
  }
  else {
    return String(digits);
  }
}

void msOverlay(OLEDDisplay *display, OLEDDisplayUiState* state) {
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->setFont(ArialMT_Plain_10);
  String timenow = String(hour())+":"+twoDigits(minute())+":"+twoDigits(second());
  display->drawString(128, 0, timenow); //String(millis())
}

void espFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  display->drawString(0 + x, 0 + y, "ESP8266");
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0 + x, 10 + y, "Uptime:");
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->drawString(128 + x, 10 + y, String(millis2time()));
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0 + x, 20 + y, "Heap:");
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->drawString(128 + x, 20 + y, String(ESP.getFreeHeap()));
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0 + x, 30 + y, "Voltage:");
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  float vccd = (ESP.getVcc());
  display->drawString(128 + x, 30 + y, String((vccd/1000),2));
}

void tempFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  display->drawString(0 + x, 0 + y, "Weather");
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0 + x, 10 + y, "Temp:");
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->drawString(128 + x, 10 + y, String(mySensor.readTempC()));
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0 + x, 20 + y, "Humidity:");
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->drawString(128 + x, 20 + y, String(mySensor.readFloatHumidity()));
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0 + x, 30 + y, "Pressure:");
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->drawString(128 + x, 30 + y, String((mySensor.readFloatPressure()/100)));
}

void hmc5883Frame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  Vector raw = compass.readRaw();
  Vector norm = compass.readNormalize();
  String rs = String(raw.XAxis) + "," + String(raw.YAxis) + "," + String(raw.ZAxis);
  String ns = String(norm.XAxis) + "," + String(norm.YAxis) + "," + String(norm.ZAxis);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  display->drawString(0 + x, 0 + y, "Compass");
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0 + x, 10 + y, "Raw:");
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->drawString(64 + x, 20 + y, rs);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0 + x, 30 + y, "Norm:");
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->drawString(64 + x, 40 + y, ns);
}

void wifiFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  display->drawString(0 + x, 0 + y, "WiFi");
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0 + x, 20 + y, "SSID:");
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  String ssid = WiFi.SSID();
  display->drawString(128 + x, 20 + y, ssid);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0 + x, 34 + y, "RSSI:");
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  String rssi = String(WiFi.RSSI());
  display->drawString(128 + x, 34 + y, rssi);
}

void analogClockFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
//  ui.disableIndicator();

  // Draw the clock face
//  display->drawCircle(clockCenterX + x, clockCenterY + y, clockRadius);
  display->drawCircle(clockCenterX + x, clockCenterY + y, 2);
  //
  //hour ticks
  for( int z=0; z < 360;z= z + 30 ){
  //Begin at 0° and stop at 360°
    float angle = z ;
    angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
    int x2 = ( clockCenterX + ( sin(angle) * clockRadius ) );
    int y2 = ( clockCenterY - ( cos(angle) * clockRadius ) );
    int x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 8 ) ) ) );
    int y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 8 ) ) ) );
    display->drawLine( x2 + x , y2 + y , x3 + x , y3 + y);
  }

  // display second hand
  float angle = second() * 6 ;
  angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
  int x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 5 ) ) ) );
  int y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 5 ) ) ) );
  display->drawLine( clockCenterX + x , clockCenterY + y , x3 + x , y3 + y);
  //
  // display minute hand
  angle = minute() * 6 ;
  angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
  x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 4 ) ) ) );
  y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 4 ) ) ) );
  display->drawLine( clockCenterX + x , clockCenterY + y , x3 + x , y3 + y);
  //
  // display hour hand
  angle = hour() * 30 + int( ( minute() / 12 ) * 6 )   ;
  angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
  x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 2 ) ) ) );
  y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 2 ) ) ) );
  display->drawLine( clockCenterX + x , clockCenterY + y , x3 + x , y3 + y);
}

void digitalClockFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  String timenow = String(hour())+":"+twoDigits(minute())+":"+twoDigits(second());
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_24);
  display->drawString(clockCenterX + x , clockCenterY + y, timenow );
}
// This array keeps function pointers to all frames
// frames are the single views that slide in
FrameCallback frames[] = {  analogClockFrame, digitalClockFrame, espFrame, tempFrame, hmc5883Frame, wifiFrame };

// how many frames are there?
int frameCount = 6;

// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback overlays[] = { msOverlay };
int overlaysCount = 1;



void setupSSD1306() {
  // The ESP is capable of rendering 60fps in 80Mhz mode
  // but that won't give you much time for anything else
  // run it in 160Mhz mode or just set it to 30 fps
  DBG_OUTPUT_PORT.printf("\nSetting up SSD1306 128x64 OLED...\n");
  DBG_OUTPUT_PORT.printf("Setting FPS to 30...\n"); 
  ui.setTargetFPS(30);
  DBG_OUTPUT_PORT.printf("Setting up OLED customisations...\n");
  ui.setActiveSymbol(activeSymbol);// Customize the active and inactive symbol
  ui.setInactiveSymbol(inactiveSymbol);
  ui.setIndicatorPosition(BOTTOM);// TOP, LEFT, BOTTOM, RIGHT
  ui.setIndicatorDirection(LEFT_RIGHT);// Defines where the first frame is located in the bar.
  ui.setFrameAnimation(SLIDE_LEFT);// SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
  ui.setFrames(frames, frameCount);// Add frames
  ui.setOverlays(overlays, overlaysCount);// Add overlays
  DBG_OUTPUT_PORT.printf("Initialising...\n");
  ui.init();// Initialising the UI will init the display too.
  display.flipScreenVertically();
  display.clear();
  DBG_OUTPUT_PORT.printf("...Done!\n");  
}

//format bytes
String formatBytes(size_t bytes){
  if (bytes < 1024){
    return String(bytes)+"B";
  } else if(bytes < (1024 * 1024)){
    return String(bytes/1024.0)+"KB";
  } else if(bytes < (1024 * 1024 * 1024)){
    return String(bytes/1024.0/1024.0)+"MB";
  } else {
    return String(bytes/1024.0/1024.0/1024.0)+"GB";
  }
}
