// Pete Dulber mods:

#define SERIAL_OUTPUT
#define LCD_OUTPUT
#define BLYNK_OUTPUT

/*
 * I2C-Generator: 0.3.0
 * Yaml Version: 2.1.3
 * Template Version: 0.7.0-112-g190ecaa
 */

/*  Pete Dubler
     Heltec WIFI-Kit32
     SDA pin 4   White Grove I2C cable
     SCL pin 15  Yellow Grove I2C cable
*/
/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.-
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <Arduino.h>
#include <heltec.h>  // 071122
#include <SensirionI2CSen5x.h>
#include <Wire.h>
//#include "string.h"  // 071122

float massConcentrationPm1p0;
float massConcentrationPm2p5;
float massConcentrationPm4p0;
float massConcentrationPm10p0;
float ambientHumidity;
float ambientTemperature;
float vocIndex;
float noxIndex;

// The used commands use up to 48 bytes. On some Arduino's the default buffer
// space is not large enough
#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

SensirionI2CSen5x sen5x;

void printModuleVersions()
{
    uint16_t error;
    char errorMessage[256];

    unsigned char productName[32];
    uint8_t productNameSize = 32;

    error = sen5x.getProductName(productName, productNameSize);

    if (error)
    {
        Serial.print("Error trying to execute getProductName(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
    else
    {
        Serial.print("ProductName:");
        Serial.println((char *)productName);
    }

    uint8_t firmwareMajor;
    uint8_t firmwareMinor;
    bool firmwareDebug;
    uint8_t hardwareMajor;
    uint8_t hardwareMinor;
    uint8_t protocolMajor;
    uint8_t protocolMinor;

    error = sen5x.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                             hardwareMajor, hardwareMinor, protocolMajor,
                             protocolMinor);
    if (error)
    {
        Serial.print("Error trying to execute getVersion(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
    else
    {
        Serial.print("Firmware: ");
        Serial.print(firmwareMajor);
        Serial.print(".");
        Serial.print(firmwareMinor);
        Serial.print(", ");

        Serial.print("Hardware: ");
        Serial.print(hardwareMajor);
        Serial.print(".");
        Serial.println(hardwareMinor);
    }
}

void printSerialNumber()
{
    uint16_t error;
    char errorMessage[256];
    unsigned char serialNumber[32];
    uint8_t serialNumberSize = 32;

    error = sen5x.getSerialNumber(serialNumber, serialNumberSize);
    if (error)
    {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
    else
    {
        Serial.print("SerialNumber:");
        Serial.println((char *)serialNumber);
    }
}

void setup()
{
    // setup Heltec WIFI KIT 32  071122
#ifdef LCD_OUTPUT
    Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/);
    //Heltec.display->flipScreenVertically();
    Heltec.display->screenRotate(ANGLE_90_DEGREE);
    Heltec.display->setFont(ArialMT_Plain_10);
    Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
#endif

    // We'll init serial in all cases just to have
    //  for misc error messages and diagnostics  071122
    Serial.begin(115200);
    while (!Serial)
    {
        delay(100);
    }


#ifdef BLYNK_OUTPUT
    // TODO  BLYNK and WIFI INIT
#endif // #ifdef BLYNK_OUTPUT

    // Wire.begin();  071122
    Wire.begin();  // for secondary IC2  071122

    sen5x.begin(Wire);

    uint16_t error;
    char errorMessage[256];
    error = sen5x.deviceReset();
    if (error)
    {
        Serial.print("Error trying to execute deviceReset(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

// Print SEN55 module information if i2c buffers are large enough
#ifdef USE_PRODUCT_INFO
    printSerialNumber();
    printModuleVersions();
#endif

    // set a temperature offset in degrees celsius
    // Note: supported by SEN54 and SEN55 sensors
    // By default, the temperature and humidity outputs from the sensor
    // are compensated for the modules self-heating. If the module is
    // designed into a device, the temperature compensation might need
    // to be adapted to incorporate the change in thermal coupling and
    // self-heating of other device components.
    //
    // A guide to achieve optimal performance, including references
    // to mechanical design-in examples can be found in the app note
    // “SEN5x – Temperature Compensation Instruction” at www.sensirion.com.
    // Please refer to those application notes for further information
    // on the advanced compensation settings used
    // in `setTemperatureOffsetParameters`, `setWarmStartParameter` and
    // `setRhtAccelerationMode`.
    //
    // Adjust tempOffset to account for additional temperature offsets
    // exceeding the SEN module's self heating.
    float tempOffset = 0.0;
    error = sen5x.setTemperatureOffsetSimple(tempOffset);
    if (error)
    {
        Serial.print("Error trying to execute setTemperatureOffsetSimple(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
    else
    {
        Serial.print("Temperature Offset set to ");
        Serial.print(tempOffset);
        Serial.println(" deg. Celsius (SEN54/SEN55 only");
    }

    // Start Measurement
    error = sen5x.startMeasurement();
    if (error)
    {
        Serial.print("Error trying to execute startMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
}

#ifdef SERIAL_OUTPUT
void PrintResultsToSerial(void)
{
    Serial.print("MassConcentrationPm1p0:");
    Serial.print(massConcentrationPm1p0);
    Serial.print("\t");
    Serial.print("MassConcentrationPm2p5:");
    Serial.print(massConcentrationPm2p5);
    Serial.print("\t");
    Serial.print("MassConcentrationPm4p0:");
    Serial.print(massConcentrationPm4p0);
    Serial.print("\t");
    Serial.print("MassConcentrationPm10p0:");
    Serial.print(massConcentrationPm10p0);
    Serial.print("\t");
    Serial.print("AmbientHumidity:");
    if (isnan(ambientHumidity))
    {
        Serial.print("n/a");
    }
    else
    {
        Serial.print(ambientHumidity);
    }
    Serial.print("\t");
    Serial.print("AmbientTemperature:");
    if (isnan(ambientTemperature))
    {
        Serial.print("n/a");
    }
    else
    {
        Serial.print(ambientTemperature);
    }
    Serial.print("\t");
    Serial.print("VocIndex:");
    if (isnan(vocIndex))
    {
        Serial.print("n/a");
    }
    else
    {
        Serial.print(vocIndex);
    }
    Serial.print("\t");
    Serial.print("NoxIndex:");
    if (isnan(noxIndex))
    {
        Serial.println("n/a");
    }
    else
    {
        Serial.println(noxIndex);
    }
}
#endif // ifdef SERIAL_OUTPUT

#ifdef LCD_OUTPUT
void DisplayResultsToLCD(void)
{
    char tempstr[20];

    Heltec.display->clear();

    sprintf(tempstr,"Temp:   %2.1f", ambientTemperature);
    Heltec.display->drawString(0   , 0  , tempstr);

    sprintf(tempstr,"Humd:   %2.1f", ambientHumidity);
    Heltec.display->drawString(0   , 16 , tempstr);

    sprintf(tempstr,"Pm1.0:  %2.2f", massConcentrationPm1p0);
    Heltec.display->drawString(0   , 32 , tempstr);

    sprintf(tempstr,"Pm2.5:  %2.2f", massConcentrationPm2p5);
    Heltec.display->drawString(0   , 48 , tempstr);

    sprintf(tempstr,"Pm4.0:  %2.2f", massConcentrationPm4p0);
    Heltec.display->drawString(0   , 64 , tempstr);

    sprintf(tempstr,"Pm10:   %2.2f", massConcentrationPm10p0);
    Heltec.display->drawString(0   , 80 , tempstr);

    sprintf(tempstr,"VOC:    %2.1f", vocIndex);
    Heltec.display->drawString(0   , 96 , tempstr);

    Heltec.display->display();
    delay(200);
}
#endif // #ifdef LCD_OUTPUT

#ifdef BLYNK_OUTPUT
void SendResultsToBlynk(void)
{
    //TODO   output data to Blynk
}
#endif // #ifdef BLYNK_OUTPUT


void loop()
{
    uint16_t error;
    char errorMessage[256];

    delay(1000);

    // Read Measurement

    error = sen5x.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);

    if (error)
    {
        Serial.print("Error trying to execute readMeasuredValues(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
    else
    {
      #ifdef SERIAL_OUTPUT
      PrintResultsToSerial();
      #endif

      #ifdef LCD_OUTPUT
      DisplayResultsToLCD();
      #endif

      #ifdef BLYNK_OUTPUT
      SendResultsToBlynk();
      #endif



    }
}