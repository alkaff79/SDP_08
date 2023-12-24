/*
we used esp33 and sim800l in one integerated circuit as a microcontroller
test each sensor
for class 1000 iso standard
temp:20-24
hum: 30-60%
*/
const char simPIN[] = "";
#define SMS_TARGET "write your phone number"//+966xxxxxxxxx
// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800    // Modem is SIM800
#define TINY_GSM_RX_BUFFER 1024  // Set RX buffer to 1Kb

#include <WiFi.h>
#include <WiFiClient.h>
#include "ThingSpeak.h"
#include "DFRobot_SHT20.h"
#include <Adafruit_BMP280.h>
#include <TinyGsmClient.h>


#include <HardwareSerial.h>

#define S0 32
#define S1 33
#define S2 14
#define Motion_pin 35
#define Gas_pin 34  //35
// TTGO T-Call pins
#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26
#define I2C_SDA 21
#define I2C_SCL 22
#define IP5306_ADDR 0x75
#define IP5306_REG_SYS_CTL0 0x00
#define PMS_TX_PIN 2
#define PMS_RX_PIN 18


// Set serial for AT commands (to SIM800 module)

#define SerialAT Serial1
// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif
int analogVal[16];
WiFiClient client;
HardwareSerial pmsSerial(2);  // Use UART 1 for TX (2) and RX (15)

DFRobot_SHT20 sht20(&Wire, 0x40);
int MotionArray[8];
int GasArray[8];
float tempreture[8];
float Humadity[8];
Adafruit_BMP280 bmp;
float pressure[8];
int i_loop = 0;
const unsigned long MOTION_DURATION[8] = { 60000, 60000, 60000, 60000, 60000, 60000, 60000, 60000 };
unsigned long motionStartTime[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };  // Variable to store the start time of motion

const unsigned long GAS_DURATION[8] = { 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000 };
unsigned long gasStartTime[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

const unsigned long TEMP_DURATION[8] = { 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000 };
unsigned long tempStartTime[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

const unsigned long HUMAD_DURATION[8] = { 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000 };
unsigned long humadStartTime[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

const unsigned long PRESS_DURATION[8] = { 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000 };
unsigned long pressStartTime[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long PMScheckTime = 0;  // Initialize a variable to hold the last time the condition was checked
const unsigned long PMScheckInterval = 10000;
const unsigned long PM03_DURATION = 60000;
unsigned long pm03StartTime = 0;
const unsigned long PM05_DURATION = 60000;
unsigned long pm05StartTime = 0;
const unsigned long PM10_DURATION = 60000;
unsigned long pm10StartTime = 0;
const unsigned long PM50_DURATION = 60000;
unsigned long pm50StartTime = 0;

unsigned long previousExecutionTime = 0;  // will store the previous execution time
const unsigned long executionInterval = 5000;
//WIFI THINGS:
const char* WIFI_NAME = "wifi_name";
const char* WIFI_PASSWORD = "wifi_pass";
const int MotionID = 2285565;
const int GasID = 2286649;
const char* MotionApiKey = "WGO5ZOGQAHN02VOB";
const char* GasApiKey = "QV4P6NJV716C6PCF";
const int TempID = 2293020;
const char* TempApiKey = "QMS12PIBF3ZOBLQB";
const int PressureID = 2293006;
const char* PressueApiKey = "Y682VRINHS17JQ6B";
const char* server = "api.thingspeak.com";
const int MOTION_THRESHOLD_LOW = 1000;// IT WAS 5 FOR BREADBOARD , and 1000 for pcb
const int MQ2_THRESHOLD = 400;  //for digital pin was 400
const int MQ4_THRESHOLD = 300;  //TESTED FOR MQ-4 HIGH 1000 AND MORE,LOW 200-100 TESTED IN PIN 34 was 300
const int MQ135_THRESHOLD = 600;
const float TempMaxRange = 27;  //should be 24
const float TemplowRange = 20;
const float HumMaxRange = 70;//70 for test it was 60
const float HumlowRange = 30;
const float PresslowRange = 700;       //should be 1000
const float PressMaxRange = 1100;      //edit it later on !!!!!!!
const uint16_t PM03_THRESHOLD = 20000;  //10200;
const uint16_t PM05_THRESHOLD = 3520;
const uint16_t PM10_THRESHOLD = 832;
const uint16_t PM50_THRESHOLD = 29;


int p_03um, p_05um, p_10um, p_50um;
struct pms9003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms9003data data;

//////////////////////////////////////////////////////////////////////////////////////////



void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}
void TCA9548A2(uint8_t bus) {
  Wire.beginTransmission(0x71);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void setup() {
  Serial.begin(9600);


  xTaskCreatePinnedToCore(
    ParticleReading,  // Function to run on the second core
    "pmSensorTask",   // Task name
    10000,            // Stack size
    NULL,             // Task parameters
    1,                // Priority
    NULL,             // Task handle
    1                 // Core to run the task (1 for the second core)
  );

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  delay(3000);

  // Restart SIM800 module, it takes quite some time
  Serial.println("Initializing modem...");
  modem.restart();

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3) {
    modem.simUnlock(simPIN);
  }



  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wifi not connected");
    delay(1000);
  }

  Serial.println("Wifi connected !");
  Serial.println("Local IP: " + String(WiFi.localIP()));
  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client);
  Wire.begin(I2C_SDA, I2C_SCL);
  bool isOk = setPowerBoostKeepOn(1);
  Serial.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));



  initBMP280Sensors();

  for (int i = 0; i < 8; i++) {
    TCA9548A2(i);

    sht20.initSHT20();
    sht20.checkSHT20();
  }
}



void loop() {

  unsigned long currentMillis = millis();

  MUXTASK();

  delay(100);

  Serial.println();


  for (int i = 0; i < 8; i++) {
    DigitalCheckMotion(i, MOTION_THRESHOLD_LOW);
    DigitalCheckGas(i);
    TempcheckRangeSHT20(tempreture[i], TemplowRange, TempMaxRange, i);
    HumaditycheckRangeSHT20(Humadity[i], HumlowRange, HumMaxRange, i);
    checkRangeBMP280(pressure[i], PresslowRange, PressMaxRange, i);
  }
  PM03SENSORTEST(p_03um, PM03_THRESHOLD);
  PM05SENSORTEST(p_05um, PM05_THRESHOLD);
  PM10SENSORTEST(p_10um, PM10_THRESHOLD);
  PM50SENSORTEST(p_50um, PM50_THRESHOLD);

  // Check if 60 seconds have passed since the last execution
  if (currentMillis - previousExecutionTime >= executionInterval) {
    // Save the current time as the previous execution time
    previousExecutionTime = currentMillis;

    // Call the function to upload values
    UploadTheValues();
  }
}


void MUXTASK(void) {

  for (int count = 0; count < 8; count++) {


    // SET THE ADDRESS
    digitalWrite(S0, bitRead(count, 0));
    digitalWrite(S1, bitRead(count, 1));
    digitalWrite(S2, bitRead(count, 2));

    int Motionreading = analogRead(Motion_pin);
    int GasReading = analogRead(Gas_pin);

    int channel = count + 1;

    Serial.print(" Gas ");
    Serial.print(channel);
    Serial.print(":");
    Serial.print(GasReading);
    Serial.println("  ");
    Serial.print(" Motion ");
    Serial.print(channel);
    Serial.print(":");
    Serial.print(Motionreading);
    Serial.println("  ");

    MotionArray[count] = Motionreading;
    GasArray[count] = GasReading;
  }


  for (int count = 0; count < 8; count++) {

    PressureReading(count);
  }
  for (int count = 0; count < 8; count++) {

    TemperatureAndHumidityReading(count);
  }
}

////////////////Values Checking////////////////
void PM03SENSORTEST(int PmReading, int maxRange) {

  if (PmReading > maxRange) {
    if (pm03StartTime == 0) {
      pm03StartTime = millis();
      //send the first SMS

      String smsMessage = "The number of particles (<0.3 um) is out range!\nValue is: " + String(PmReading) + "um/0.1L";
      if (modem.sendSMS(SMS_TARGET, smsMessage)) {
        Serial.println(smsMessage);
      } else {
        Serial.println("SMS failed to send");
      }
    }
    if (millis() - pm03StartTime >= PM03_DURATION) {
      String smsMessage1 = "The number of particles (<0.3 um) is still out range!!\nValue is: " + String(PmReading) + "um/0.1L";
      if (modem.sendSMS(SMS_TARGET, smsMessage1)) {
        Serial.println(smsMessage1);
      } else {
        Serial.println("SMS failed to send1");
      }
    }

  } else {
    pm03StartTime = 0;
  }
}
void PM05SENSORTEST(int PmReading, int maxRange) {

  if (PmReading > maxRange) {
    if (pm05StartTime == 0) {
      pm05StartTime = millis();
      //send the first SMS

      String smsMessage = "The number of particles (<0.5 um) is out range!\nValue is: " + String(PmReading) + "um/0.1L";
      if (modem.sendSMS(SMS_TARGET, smsMessage)) {
        Serial.println(smsMessage);
      } else {
        Serial.println("SMS failed to send");
      }
    }
    if (millis() - pm05StartTime >= PM05_DURATION) {
      String smsMessage1 = "The number of particles (<0.5 um) is still out range!!\nValue is: " + String(PmReading) + "um/0.1L";
      if (modem.sendSMS(SMS_TARGET, smsMessage1)) {
        Serial.println(smsMessage1);
      } else {
        Serial.println("SMS failed to send1");
      }
    }

  } else {
    pm05StartTime = 0;
  }
}

void PM10SENSORTEST(int PmReading, int maxRange) {

  if (PmReading > maxRange) {
    if (pm10StartTime == 0) {
      pm10StartTime = millis();
      //send the first SMS

      String smsMessage = "The number of particles (< 1 um) is out range!\nValue is: " + String(PmReading) + "um/0.1L";
      if (modem.sendSMS(SMS_TARGET, smsMessage)) {
        Serial.println(smsMessage);
      } else {
        Serial.println("SMS failed to send");
      }
    }
    if (millis() - pm10StartTime >= PM10_DURATION) {
      String smsMessage1 = "The number of particles (< 1 um) is still out range!!\nValue is: " + String(PmReading) + "um/0.1L";
      if (modem.sendSMS(SMS_TARGET, smsMessage1)) {
        Serial.println(smsMessage1);
      } else {
        Serial.println("SMS failed to send1");
      }
    }

  } else {
    pm10StartTime = 0;
  }
}

void PM50SENSORTEST(int PmReading, int maxRange) {

  if (PmReading > maxRange) {
    if (pm50StartTime == 0) {
      pm50StartTime = millis();
      //send the first SMS

      String smsMessage = "The number of particles (< 5 um) is out range!\nValue is: " + String(PmReading) + "um/0.1L";
      if (modem.sendSMS(SMS_TARGET, smsMessage)) {
        Serial.println(smsMessage);
      } else {
        Serial.println("SMS failed to send");
      }
    }
    if (millis() - pm50StartTime >= PM50_DURATION) {
      String smsMessage1 = "The number of particles (< 5 um) is still out range!!\nValue is: " + String(PmReading) + "um/0.1L";
      if (modem.sendSMS(SMS_TARGET, smsMessage1)) {
        Serial.println(smsMessage1);
      } else {
        Serial.println("SMS failed to send1");
      }
    }

  } else {
    pm50StartTime = 0;
  }
}

void TempcheckRangeSHT20(float TempReading, float minRangeT, float maxRangeT, int i) {

  if (TempReading < minRangeT || TempReading > maxRangeT) {
    if (TempReading != 0) {
      if (tempStartTime[i] == 0) {
        tempStartTime[i] = millis();
        //send the first SMS

        String smsMessage = "Temperature is out range in channel:" + String(i + 1) + "\nValue is: " + String(TempReading);
        if (modem.sendSMS(SMS_TARGET, smsMessage)) {
          Serial.println(smsMessage);
        } else {
          Serial.println("SMS failed to send");
        }
      }
      if (millis() - tempStartTime[i] >= TEMP_DURATION[i]) {
        String smsMessage1 = "Temperature still out range in channel: " + String(i + 1);
        if (modem.sendSMS(SMS_TARGET, smsMessage1)) {
          Serial.println(smsMessage1);
        } else {
          Serial.println("SMS failed to send1");
        }
      }

    } else {
      tempStartTime[i] = 0;
    }
  }
}
void HumaditycheckRangeSHT20(float HUMADReading, float minRangeH, float maxRangeH, int i) {

  if (HUMADReading < minRangeH || HUMADReading > maxRangeH) {
    if (HUMADReading != 0) {

      if (humadStartTime[i] == 0) {
        humadStartTime[i] = millis();
        //send the first SMS
        String smsMessage = "Humidity is outside range in channel:" + String(i + 1) + "\nValue is: " + String(HUMADReading);
        if (modem.sendSMS(SMS_TARGET, smsMessage)) {
          Serial.println(smsMessage);
        } else {
          Serial.println("SMS failed to send");
        }
      }
      if (millis() - humadStartTime[i] >= HUMAD_DURATION[i]) {
        String smsMessage1 = "Humidity still outside range in channel: " + String(i + 1);

        if (modem.sendSMS(SMS_TARGET, smsMessage1)) {
          Serial.println(smsMessage1);
        } else {
          Serial.println("SMS failed to send1");
        }
      }

    } else {
      humadStartTime[i] = 0;
    }
  }
}


void checkRangeBMP280(float PressureReading, float minRange, float maxRange, int i) {
  if (PressureReading < minRange || PressureReading > maxRange) {
    if (PressureReading != 0) {

      if (pressStartTime[i] == 0) {
        pressStartTime[i] = millis();
        //send the first SMS

        String smsMessage = "Pressure is outside range in channel:" + String(i + 1) + "\nValue is: " + String(PressureReading);
        if (modem.sendSMS(SMS_TARGET, smsMessage)) {
          Serial.println(smsMessage);
        } else {
          Serial.println("SMS failed to send");
        }
      }
      if (millis() - pressStartTime[i] >= PRESS_DURATION[i]) {
        String smsMessage1 = "Pressure still outside range in channel: " + String(i + 1);

        // modem.sendSMS(SMS_TARGET, smsMessage1);
        if (modem.sendSMS(SMS_TARGET, smsMessage1)) {
          Serial.println(smsMessage1);
        } else {
          Serial.println("SMS failed to send1");
        }
      }

    } else {
      pressStartTime[i] = 0;
    }
  }
}


void continuousMovmentCHK(int i, int Threshold, int sensorReading) {

  if (sensorReading <= Threshold) {//make it >= and change the threshold for breadboard

    if (motionStartTime[i] == 0) {
      motionStartTime[i] = millis();
      //send the first SMS
      String smsMessage = "Motion has been detected in channel:" + String(i + 1);
      delay(300);
      if (modem.sendSMS(SMS_TARGET, smsMessage)) {
        Serial.println(smsMessage);
      } else {
        Serial.println("SMS failed to send");
      }
    }
    if (millis() - motionStartTime[i] >= MOTION_DURATION[i]) {

      String smsMessage1 = "Motion still detected in channel: " + String(i + 1);
      if (modem.sendSMS(SMS_TARGET, smsMessage1)) {
        Serial.println(smsMessage1);
      } else {
        Serial.println("SMS failed to send1");
      }
    }

  } else {
    motionStartTime[i] = 0;
  }
}

void DigitalCheckMotion(int i, int Threshold) {

  int sensorReading = MotionArray[i];

  switch (i) {
    case 0:
      continuousMovmentCHK(i, Threshold, sensorReading);

      break;

    case 1:
      continuousMovmentCHK(i, Threshold, sensorReading);

      break;

    case 2:
      continuousMovmentCHK(i, Threshold, sensorReading);

      // Handle case 2
      // Code for case 2 goes here
      break;

    case 3:
      continuousMovmentCHK(i, Threshold, sensorReading);
      break;

    case 4:
      continuousMovmentCHK(i, Threshold, sensorReading);

      break;

    case 5:
      continuousMovmentCHK(i, Threshold, sensorReading);

      break;

    case 6:
      continuousMovmentCHK(i, Threshold, sensorReading);

      break;

    case 7:
      continuousMovmentCHK(i, Threshold, sensorReading);

      break;
  }
}

void continuousGasCHK(int sensorReading, int Threshold, int i) {

  if (sensorReading > Threshold) {

    if (gasStartTime[i] == 0) {
      gasStartTime[i] = millis();
      //send the first SMS

      String smsMessage = "Gas leakage detected in channel:" + String(i + 1);
      modem.sendSMS(SMS_TARGET, smsMessage);
      if (modem.sendSMS(SMS_TARGET, smsMessage)) {
        Serial.println(smsMessage);
      } else {
        Serial.println("SMS failed to send");
      }
    }
    if (millis() - gasStartTime[i] >= GAS_DURATION[i]) {
      String smsMessage1 = "Gas leakage still detected in channel: " + String(i + 1);

      if (modem.sendSMS(SMS_TARGET, smsMessage1)) {
        Serial.println(smsMessage1);
      } else {
        Serial.println("SMS failed to send1");
      }
    }

  } else {
    gasStartTime[i] = 0;
  }
}

void DigitalCheckGas(int i) {
  int sensorReading = GasArray[i];

  switch (i) {
    case 0:
      continuousGasCHK(sensorReading, MQ2_THRESHOLD, 0);

      break;

    case 1:
      continuousGasCHK(sensorReading, MQ4_THRESHOLD, 1);

      break;

    case 2:
      continuousGasCHK(sensorReading, MQ135_THRESHOLD, 2);

      // Handle case 2
      // Code for case 2 goes here
      break;
  }
}



////////////////Values Uploading////////////////

void MotionAndGasUpload(int* dataArray, int startNum, int numElements, int channelID, const char* apiKey) {
  for (int i = startNum; i < startNum + numElements; i++) {
    int fieldNum = i + 1;                                    // Incrementing i by 1 to start from 1 instead of 0
    ThingSpeak.setField(fieldNum, dataArray[i - startNum]);  // Setting the field value for ThingSpeak
  }
  ThingSpeak.writeFields(channelID, apiKey);  // Sending the data to ThingSpeak
}
void PMSUPload1(int dataArray, unsigned int startNum, int numElements, int channelID, const char* apiKey) {
  for (int i = startNum; i < numElements; i++) {
    // int fieldNum = i + 1;                      // Incrementing i by 1 to start from 1 instead of 0
    ThingSpeak.setField(startNum, dataArray);  // Setting the field value for ThingSpeak
  }
  ThingSpeak.writeFields(channelID, apiKey);  // Sending the data to ThingSpeak
}
void PressureUpload(float* dataArray, int startNum, int numElements, int channelID, const char* apiKey) {
  for (int i = startNum; i < startNum + numElements; i++) {
    int fieldNum = i + 1;                                    // Incrementing i by 1 to start from 1 instead of 0
    ThingSpeak.setField(fieldNum, dataArray[i - startNum]);  // Setting the field value for ThingSpeak
  }
  ThingSpeak.writeFields(channelID, apiKey);  // Sending the data to ThingSpeak
}
void TempAndHumUpload(float* dataArray1, float* dataArray2, int startNum, int numElements, int channelID, const char* apiKey) {
  for (int i = startNum; i < startNum + numElements; i++) {
    int fieldNum = i + 1;  // Incrementing i by 1 to start from 1 instead of 0
    int fieldNum1 = i + 5;
    ThingSpeak.setField(fieldNum, dataArray1[i - startNum]);  // Setting the field value for ThingSpeak
    ThingSpeak.setField(fieldNum1, dataArray2[i - startNum]);
  }
  ThingSpeak.writeFields(channelID, apiKey);  // Sending the data to ThingSpeak
}

////////////////Call The Uploading Function////////////////
void UploadTheValues(void) {
  // void onlineTaskk(int* dataArray, int numElements, int startNum, int channelID, const char* apiKey)
  MotionAndGasUpload(MotionArray, 0, 8, MotionID, MotionApiKey);
  MotionAndGasUpload(GasArray, 0, 3, GasID, GasApiKey);

  PMSUPload1(p_03um, 4, 5, GasID, GasApiKey);

  PMSUPload1(p_05um, 5, 6, GasID, GasApiKey);
  PMSUPload1(p_10um, 6, 7, GasID, GasApiKey);
  PMSUPload1(p_50um, 7, 8, GasID, GasApiKey);
  // PMUpload(p_03um, 4, GasID, GasApiKey);
  // PMUpload(p_05um, 5, GasID, GasApiKey);
  // PMUpload(p_10um, 6, GasID, GasApiKey);
  // PMUpload(p_50um, 7, GasID, GasApiKey);



  TempAndHumUpload(tempreture, Humadity, 0, 4, TempID, TempApiKey);
  PressureUpload(pressure, 0, 8, PressureID, PressueApiKey);
}

////////////////Read & Store values from sensors////////////////

void PressureReading(int bus) {
  TCA9548A(bus);
  if (isBMP280Present()) {
    Serial.print(bus + 1);
    Serial.print(" pressure is:");
    Serial.println(bmp.readPressure() / 100.0F);

    pressure[bus] = (bmp.readPressure() / 100.0F);
  }
}

void TemperatureAndHumidityReading(int bus) {

  TCA9548A2(bus);

  float missing = 998;
  float value = 0.0 / 0.0;
  float humd = sht20.readHumidity();
  float temp = sht20.readTemperature();
  Serial.print(bus);

  Serial.print(" Temperature:");



  if (temp == missing) {
    temp = value;
    Serial.print(temp);
  } else {
    tempreture[bus] = temp;

    Serial.print(temp, 1);
  }
  Serial.print("C");
  Serial.print(" Humidity:");
  if (humd == missing) {
    humd = value;
    Serial.print(humd);
  } else {
    Serial.print(humd, 1);
    Humadity[bus] = humd;
  }
  Serial.print("%");
  Serial.println();
  Serial.println();
}

void ParticleReading(void* parameter) {
  for (;;) {
    if (readPMSdata(&pmsSerial)) {
      // Reading data was successful!
      p_03um = data.particles_03um;
      p_05um = data.particles_05um;
      p_10um = data.particles_10um;
      p_50um = data.particles_50um;
    delay(20000);

      Serial.println("---------------------------------------");
      Serial.print("Particles > 0.3um / 0.1L air:");
      Serial.println(p_03um);
      Serial.print("Particles > 0.5um / 0.1L air:");
      Serial.println(p_05um);
      Serial.print("Particles > 1.0um / 0.1L air:");
      Serial.println(p_10um);
      Serial.print("Particles > 5.0um / 0.1L air:");
      Serial.println(p_50um);
      Serial.println("---------------------------------------");
      // Serial.print(iloop);

      delay(10000);
    }

    // Add any other functionality you want to run on the second core here

    // You can add a delay here to control the rate at which this task executes
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
  }
}
boolean readPMSdata(Stream* s) {
  if (!s->available()) {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void*)&data, (void*)buffer_u16, 30);
 

  if (sum != data.checksum) {
    
    Serial.println("Checksum failure");
    Serial.println("Dumping PMS data:");
    for (uint8_t i = 0; i < 32; i++) {
      Serial.print("0x");
      Serial.print(buffer[i], HEX);
      Serial.print(", ");
    }
    Serial.println();
  }
  // success!
  return true;
}

////////////////Initilzatin////////////////

bool isBMP280Present() {
  Wire.beginTransmission(0x76);
  return (Wire.endTransmission() == 0);
}


void initBMP280Sensors() {
  Wire.beginTransmission(0x70);
  for (int i = 0; i < 4; i++) {
    Wire.write(1 << i);
  }
  Wire.endTransmission();

  for (int i = 0; i < 4; i++) {
    TCA9548A(i);
    if (isBMP280Present()) {
      if (bmp.begin(0x76)) {
        Serial.print("BMP280 sensor found on channel ");
        Serial.println(i);
      } else {
        Serial.print("Error initializing BMP280 sensor on channel ");
        Serial.println(i);
      }
    }
  }
}


bool setPowerBoostKeepOn(int en) {
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  if (en) {
    Wire.write(0x37);  // Set bit1: 1 enable 0 disable boost keep on
  } else {
    Wire.write(0x35);  // 0x37 is default reg value
  }
  return Wire.endTransmission() == 0;
}




char checkValue(unsigned char* thebuf, char leng) {
  char receiveflag = 0;
  int receiveSum = 0;

  for (int i = 0; i < (leng - 2); i++) {
    receiveSum = receiveSum + thebuf[i];
  }
  receiveSum = receiveSum + 0x42;

  if (receiveSum == ((thebuf[leng - 2] << 8) + thebuf[leng - 1]))  //check the serial data
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}