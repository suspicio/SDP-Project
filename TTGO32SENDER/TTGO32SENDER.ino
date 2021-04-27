#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "mbedtls/aes.h"
#include "mbedtls/rsa.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

extern "C" {
#include "crypto/base64.h"
}


#define GPS_BAND_RATE      9600
#define LORA_BAND 868E6

#define LORA_MISO 19
#define LORA_CS 18
#define LORA_MOSI 27
#define LORA_SCK 5
#define LORA_RST 14
#define LORA_IRQ 26

SFE_UBLOX_GNSS myGPS;
#define GPS_RX_PIN 34
#define GPS_TX_PIN 12
#define BUTTON_PIN 38
#define BUTTON_PIN_MASK GPIO_SEL_38
// LoRa Band (change it if you are outside Europe according to your country)


MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

void setup() {
  Serial.begin(9600);
  Serial.println("Setup LoRa Sender....");
  initLoRa();
  initHBSensor();
  initGPS();
}
bool x = true;
bool y = true;
long latitude, longitude, altitude; uint8_t SIV;
uint16_t counter = 0;
void loop() {
  char  key[] = "secretkeyeaes128";
  getGPSData();
  getHBData();
  char plainbuf[200];
  char encbuf[200];
  unsigned char * base64buf;
  const char * str;
  snprintf(plainbuf, sizeof(plainbuf), "{\"i\":%d,\"h\":%d,\"o\":%d,\"hf\":%d,\"sf\":%d,\"gps\":[%li,%li,%li]}", 43, heartRate, spo2, validHeartRate, validSPO2,latitude,longitude,altitude);
  //cipher->setKey(key);
  size_t outputLength;
  //str = cipher->encryptString(plainbuf).c_str();
  //base64buf = base64_encode((const unsigned char *)str, strlen(str), &outputLength);
  LoRa.beginPacket();
  LoRa.print(plainbuf);
  LoRa.endPacket();
  Serial.print("DATA: "); Serial.println(plainbuf);
  y = true;
  free(base64buf);
}
void initGPS() {
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  if (myGPS.begin(Serial1)) {
    Serial.print("GPS MODULE RESPONDS!");
  } else {
    Serial.print("GPS MODULE DOES NOT RESPOND!");
  }
}
void getGPSData() {
  longitude = myGPS.getLongitude();
  latitude = myGPS.getLatitude();
  altitude = myGPS.getAltitude();
  SIV = myGPS.getSIV();


}
void initLoRa() {

  Serial.println("Initializing LoRa....");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  // Start LoRa using the frequency
  int result = LoRa.begin(LORA_BAND);
  if (result != 1) {
    Serial.println("Failed to start LoRa network!");
    for (;;);
  }
  Serial.println("LoRa initialized");
  delay(2000);
}
void getHBData() {
  if (x) {
    bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

    //read the first 100 samples, and determine the signal range
    for (byte i = 0 ; i < bufferLength ; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      /* Serial.print(F("red="));
        Serial.print(redBuffer[i], DEC);
        Serial.print(F(", ir="));
        Serial.println(irBuffer[i], DEC);*/
    }

    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    x = false;
  }
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (y)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      //      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      /* Serial.print(F("red="));
        Serial.print(redBuffer[i], DEC);
        Serial.print(F(", ir="));
        Serial.print(irBuffer[i], DEC);

        Serial.print(F(", HR="));
        Serial.print(heartRate, DEC);

        Serial.print(F(", HRvalid="));
        Serial.print(validHeartRate, DEC);

        Serial.print(F(", SPO2="));
        Serial.print(spo2, DEC);

        Serial.print(F(", SPO2Valid="));
        Serial.println(validSPO2, DEC);*/
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    y = false;
  }


}
void initHBSensor() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

void encryptData() {
}
