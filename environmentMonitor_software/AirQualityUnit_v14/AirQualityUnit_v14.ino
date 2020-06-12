
/*

   Some links:
   https://github.com/lewapek/sds-dust-sensors-arduino-library
   https://gist.github.com/marw/9bdd78b430c8ece8662ec403e04c75fe
  Fermentation Friend

  This sketch demonstrates the capabilities of the pubsub library in combination
  with the ESP8266 board/library.

  It connects to WiFi (or creates a hotspot to enter in the Wifi credentials if needed)

  It displays either: all sensor data, or a large version of each sensor attached and a scrolling graph of the data
  Scroll through this using the encoder.

  It publishes the sensor values via MQTT to the broker in the config (adafruit IO in this example).

  Dust sensor is an Nova Fitness SDS011, SDS021.

  Temperature sensor is

  It will reconnect to the server if the connection is lost using a blocking
  reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
  achieve the same result without blocking the main loop.

  To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"

  You need to include the following libraries, via the library manager in Arduino
    WiFiManager (v 0.15.0) by tzapu
    Adafruit_NeoPixel by Adafruit
    Adafruit_MQTT_Library by Adafruit
    U8g2 by Oliver
    Button2 by Lennart Hennings
    SdsDustSensor by:https://github.com/lewapek/sds-dust-sensors-arduino-library

*/

// Config.h includes all the hardware and defines for the board
#include "Config.h"

// For temperature sensor TH02
#include <TH02_dev.h>
#include "Wire.h"     // This is also needed for the OLED screen

// For Dust Sensor
#include <SoftwareSerial.h>
#include "SdsDustSensor.h"

// For the BMP180 sensor
#include <BMP180I2C.h>
#define I2C_ADDRESS 0x77
//create an BMP180 object using the I2C interface
BMP180I2C bmp180(I2C_ADDRESS);

SoftwareSerial mySerial(SW_RX, SW_TX); // RX, TX
SdsDustSensor sds(mySerial); // passing HardwareSerial& as parameter

SoftwareSerial radSerial(SW_RX_RAD, SW_TX_RAD); // RX, TX

// ************** WIFI Libraries ************************************

#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// ********** For the RGB LEDS (Neopixels) *************************
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixels(RGBLED_COUNT, RGBLED_DATA_PIN , RGBLED_TYPE);

// ********** For the I2C OLED Screen and Graphics *****************
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
CBOLED_CLASS u8g2(U8G2_R0, CBOLED_SCK_PIN, CBOLED_SDA_PIN, U8X8_PIN_NONE);

// *********** For the BUTTONS AND ENCODERS *************************
#include "Button2.h"; //  https://github.com/LennartHennigs/Button2
#include "ESPRotary.h";


// ********* For the ADAFRUIT IO MQTT CONNECTION **********************/
// WiFiFlientSecure for SSL/TLS support
WiFiClientSecure client;
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
// io.adafruit.com SHA1 fingerprint
static const char *fingerprint PROGMEM = "77 00 54 2D DA E7 D8 03 27 31 23 99 EB 27 DB CB A5 4C 57 18";

/************* Feeds for Adafruit IO *********************************/
// Setup a feed called 'test' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
//Adafruit_MQTT_Publish fermenterTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/fermenterTemp");
Adafruit_MQTT_Publish airTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airTemp");
Adafruit_MQTT_Publish airPM = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airPM");
Adafruit_MQTT_Publish airPM10 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airPM10");
Adafruit_MQTT_Publish airHumidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airHumidity");
Adafruit_MQTT_Publish airPressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airPressure");
Adafruit_MQTT_Publish airRadiation = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airRadiation");

//// ********* For other MQTT Connection - Read Cheers Lights *********
//#include <PubSubClient.h>
//PubSubClient client(client);  // Old library (but nice!)

/******************** Sketch Code ************************************/

byte displayMode = 100;    // Holds the page to display - Start in Start-up screen
byte maxDisplayMode = 1 + (NUMBER_SENSORS * 2); // Roll around the modes Main screen (1) plus 2 for each sensor

bool updateMQTTflag = false;

bool wificonnect = false; // Flags for the display
bool mqttconnect = false; // Flags for the display

float temp1;      // Holds the air temperature
float pm2_5;      // Holds the PM2.5 value
float pm10;       // Holds the PM10 value
float humidity;   // Holds the humidity value
float barPressure;   // Holds the barometric pressure value
float radiation;

float tempHigh = 20.0;
float tempLow = 10.0;

int   AveCounter = 0;  // Holds the number of samples read to create the average value
int   PMAveCounter = 0; // Only average the PM values when they are updated

float temp1Max = -999.9;    // Start low
float temp1Min = 999.9;     // Start High
float temp1Ave = 0.0;

float pm2_5Max = -999.9;    // Start low
float pm2_5Min = 999.9;     // Start High
float pm2_5Ave = 0.0;

float pm10Max = -999.9;    // Start low
float pm10Min = 999.9;     // Start High
float pm10Ave = 0.0;

float humidityMax = -999.9;    // Start low
float humidityMin = 999.9;     // Start High
float humidityAve = 0.0;

float barPressureAve = 0.0;

float radiationMax = -999.99;
float radiationMin = 999.99;
float radiationAve = 0.0;
int radiationCounter = 0; // For averaging the radation sensor

int sensorMQTTSend  = 0;  // This is the sensor to send to MQTT 1/number of sensors = time between each MQTT write

long int dataCounterTime = DATA_AVERAGE * 1000;   // Holds mS until next data upload
long int graphCounterTime = GRAPH_AVERAGE * 1000;  // Holds ms until next graph point
long int displayCounterTime = DISPLAY_UPDATE;   // How often to check data and update display (mS)
long int radiationCounterTime = 1000;  // Control counter to radiation sensor

long int sdsCounterTime = 0;      // Holds mS until next sds check
boolean sdsAwake = false;
boolean sdsSend = false;    // Only update MQTT when a value has been taken (so "0.00" does not get sent)
int PMcounter = 0;        // This means both PM2.5 and PM10 readings are sent to Adafruit IO

// Graph drawing inputs
#define sizeOfBuffer 120
float temp1Buffer[sizeOfBuffer];  // Sets up a buffer of floats for displaying data
float pm2_5Buffer[sizeOfBuffer];  // Sets up a buffer of floats for displaying data
float pm10Buffer[sizeOfBuffer];  // Sets up a buffer of floats for displaying data
float humidityBuffer[sizeOfBuffer];  // Sets up a buffer of floats for displaying data
float barPressureBuffer[sizeOfBuffer];  // Sets up a buffer of floats for displaying data
float radiationBuffer[sizeOfBuffer];  // Sets up a buffer of floats for displaying data

int startx = 2;          // For drawing the graphs (bottom left corner)
int starty = 46;          // For drawing the graphs (bottom left corner)
float graphHeight = 30.0;     // Height of graph (its 120 pixels wide, 64 pixels high)
float graphHeightMaxValue = 30.0;     // Temp Graph Maximum of the int values to be displayed
float graphHeightMinValue = 10.0;     // Temp Graph Minimum of the int values to be displayed
float graphPM25HeightMaxValue = 100;     // PM2.5 Graph Maximum of the int values to be displayed
float graphPM25HeightMinValue = 0;     // PM2.5 Graph Minimum of the int values to be displayed
float graphPM10HeightMaxValue = 300;     // PM2.5 Graph Maximum of the int values to be displayed
float graphPM10HeightMinValue = 0;     // PM2.5 Graph Minimum of the int values to be displayed
float graphHumidityHeightMaxValue = 300;     // PM2.5 Graph Maximum of the int values to be displayed
float graphHumidityHeightMinValue = 0;     // PM2.5 Graph Minimum of the int values to be displayed
float graphbarPressureHeightMaxValue = 100000;     // PM2.5 Graph Maximum of the int values to be displayed
float graphbarPressureHeightMinValue = 98000;     // PM2.5 Graph Minimum of the int values to be displayed
float graphRadiationHeightMaxValue = 50;     // PM2.5 Graph Maximum of the int values to be displayed
float graphRadiationHeightMinValue = 0;     // PM2.5 Graph Minimum of the int values to be displayed

float graphCalculatedY;   // for doing calculations

ESPRotary r = ESPRotary(ROT_A_PIN2, ROT_B_PIN2, (int)4);
Button2 b = Button2(ROT_PUSH_PIN2);

void setup()
{
  Serial.begin(115200);
  EEPROM.begin(10);

  Wire.begin(SW_SDA, SW_SCK); // Start the I2C comms with different pins for the sensors

  delay(150);
  /* Reset HP20x_dev */
  TH02.begin();
  delay(100);

  // Setup the BMP180 sensor
  //begin() initializes the interface, checks the sensor ID and reads the calibration parameters.
  if (!bmp180.begin())
  {
    Serial.println("begin() failed. check your BMP180 Interface and I2C Address.");
    while (1);
  }
  //reset sensor to default parameters.
  bmp180.resetToDefaults();
  //enable ultra high resolution mode for pressure measurements
  bmp180.setSamplingMode(BMP180MI::MODE_UHR);

  radSerial.begin(9600);  // Start the radiation sensor recording


  pixels.begin();
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.show();   // Send the updated pixel colors to the hardware.

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.sendBuffer();

  // Show display
  updateScreen(displayMode, wificonnect, mqttconnect);

  // Sort out Wifi
  setup_wifi();

  updateScreen(displayMode, wificonnect, mqttconnect); // Show if connected or not

  // check the fingerprint of io.adafruit.com's SSL cert
  client.setFingerprint(fingerprint);

  // Sort out the sioftware serial at 9600 to talk to the SDS sensor
  mySerial.begin(9600);
  sds.begin(); // this line will begin soft_serial with given baud rate (9600 by default)

  // Init the RotaryInput object
  r.setChangedHandler(rotate);
  b.setLongClickHandler(click);

  // Initialise the temperature Buffers
  for (int i = 0; i < sizeOfBuffer; i++)
  {
    temp1Buffer[i] = 0;
    pm2_5Buffer[i] = 0;
    pm10Buffer[i] = 0;
    humidityBuffer[i] = 0;
    barPressureBuffer[i] = 0;
    radiationBuffer[i] = 0;
  }
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(180);
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("AutoConnectAP")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();  // Reset of cannot connect - got to AP mode
    delay(5000);
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  wificonnect = true;
  displayMode = EEPROM.read(10);  // After showing - update to mode
}

uint32_t x = 0;
float value = 0.0;  // Holds the temperature value to display and to send to AdafruitIO

void loop() {
  // This is the main loop
  // Check the input encoder and button:
  r.loop();
  b.loop();


  // Wake up the SDS dust sensor ?
  if ((millis() >= (sdsCounterTime + ((SDS_UPDATE * 1000) - (SDS_WAKEUPTIME * 1000)))) && (sdsAwake == false))
  {
    // Enter here before the
    sdsCounterTime = millis(); // Reset the counter
    sdsAwake = true;
    sds.wakeup();
  }

  // Check the SDS sensor?
  if (millis() >= sdsCounterTime + SDS_UPDATE * 1000)
  {
    // Enter here when the counter is over the update time
    sdsCounterTime = millis(); // Reset the counter
    PmResult pm = sds.queryPm();
    if (pm.isOk()) {
      pm2_5 = pm.pm25;
      pm10 = pm.pm10;
      if (DEBUG_SDS == 1)
      {
        Serial.print("PM2.5 = ");
        Serial.print(pm.pm25);
        Serial.print(", PM10 = ");
        Serial.println(pm.pm10);
        // if you want to just print the measured values, you can use toString() method as well
        Serial.println(pm.toString());
      }
    } else {
      if (DEBUG_SDS == 1)
      {
        Serial.print("Could not read values from sensor, reason: ");
        Serial.println(pm.statusToString());
      }
    }
    WorkingStateResult state = sds.sleep();
    if (state.isWorking()) {
      if (DEBUG_SDS == 1)
      {
        Serial.println("Problem with sleeping the sensor.");
      }

    } else {
      if (DEBUG_SDS == 1)
      {
        Serial.println("Sensor is sleeping");
      }
    }
    sdsSend = true; // send data via MQTT
    sdsAwake = false; // It will go back to sleep
  }

  if (millis() >= radiationCounterTime + 3000)
  {
    radiationCounterTime = millis();  // Come here every 3 seconds
    radSerial.listen();
    int inByte, oldInByte;
    if (DEBUG_RAD == 1)
    {
      Serial.print(F("Radiation Data:"));
    }
    while (radSerial.available() > 0) {
      oldInByte = radSerial.parseInt();
      if (oldInByte == 0 || oldInByte > 99999)
      {
        break;
      }
      inByte = oldInByte;   // Only want the last data point NOT equal to zero...
    }
    if (DEBUG_RAD == 1)
    {
      Serial.print(inByte);
      Serial.println("");
    }
    if (radiation < 99999)
    {
      // This stops the 'blank' value being written
      radiationCounter++; // Increment the radiation Averaging
      radiationAve += inByte; // Add the value to radiation averaging
      radiation = inByte;
      mySerial.listen();
      if (radiation > radiationMax)
      {
        radiationMax = radiation;
      }
      if (radiation < radiationMin)
      {
        radiationMin = radiation;
      }
    }
  }

  // ****** Update the display *********************
  if (millis() >= (displayCounterTime + (DISPLAY_UPDATE)))
  {
    displayCounterTime = millis(); // Store new value for next time

    // Check temperature and humidty via TH02 Sensor on I2C
    temp1 = TH02.ReadTemperature();
    humidity = TH02.ReadHumidity();

    //Check the pressure readings through the BMP180 sensor
    //start a temperature measurement
    if (!bmp180.measureTemperature())
    {
      if (DEBUG_BMP180 == 1)
      {
        Serial.println("BMP180 Err - Temp");
      }
      return;
    }

    //wait for the measurement to finish. proceed as soon as hasValue() returned true.
    do
    {
      delay(100);
    } while (!bmp180.hasValue());
    if (DEBUG_BMP180 == 1)
    {
      Serial.print("BMP180 Temp: ");
      Serial.print(bmp180.getTemperature());
      Serial.print(" C");
    }
    //start a pressure measurement. pressure measurements depend on temperature measurement, you should only start a pressure
    //measurement immediately after a temperature measurement.
    if (!bmp180.measurePressure())
    {
      if (DEBUG_BMP180 == 1)
      {
        Serial.println("BMP180 Err - Pressure");
      }
      return;
    }
    //wait for the measurement to finish. proceed as soon as hasValue() returned true.
    do
    {
      delay(100);
    } while (!bmp180.hasValue());

    barPressure = bmp180.getPressure();

    if (DEBUG_BMP180 == 1)
    {
      Serial.print("Pressure: ");
      Serial.print(barPressure);
      Serial.println(" Pa");
    }
    barPressure = bmp180.getPressure();

    // End pressure measurements

    if (DEBUG_TEMP == 1)
    {
      Serial.print("Temp: ");
      Serial.print(temp1);
      Serial.print("C ");
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.println("%\r\n");
    }

    // Here we do some statistics on the data:
    if (temp1 > temp1Max && temp1 < 120.0)
    {
      temp1Max = temp1;
    }
    if (temp1 < temp1Min && temp1 > -100.0)
    {
      temp1Min = temp1;
    }
    temp1Ave += temp1;

    if (sdsSend == true)
    {
      if (pm2_5 > pm2_5Max)
      {
        pm2_5Max = pm2_5;
      }
      if (pm2_5 < pm2_5Min)
      {
        pm2_5Min = pm2_5;
      }
      pm2_5Ave += pm2_5;

      if (pm10 > pm10Max)
      {
        pm10Max = pm10;
      }
      if (pm10 < pm10Min)
      {
        pm10Min = pm10;
      }
      pm10Ave += pm10;
      PMAveCounter++;
    }
    if (humidity > humidityMax)
    {
      humidityMax = humidity;
    }
    if (humidity < humidityMin)
    {
      humidityMin = humidity;
    }
    humidityAve += humidity;



    barPressureAve += barPressure;
    AveCounter++;
    // ****** DISPLAY THE VALUE **********************
    // Here want to update the display with the value:
    updateScreen(displayMode, wificonnect, mqttconnect);
  }

  // ******* graph buffer update *************
  // Only do this when over the graph update time
  if (millis() >= (graphCounterTime + (GRAPH_AVERAGE * 1000)))
  {

    graphCounterTime = millis(); // Store new value for next time
    // Sort out the display buffer
    for (int z = (sizeOfBuffer - 2); z >= 0; z--)
    {
      // Shift all the values along
      temp1Buffer[z + 1] = temp1Buffer[z];
      if (sdsSend == true)
      {
        pm2_5Buffer[z + 1] = pm2_5Buffer[z];
        pm10Buffer[z + 1] = pm10Buffer[z];
      }
      humidityBuffer[z + 1] = humidityBuffer[z];
      barPressureBuffer[z + 1] = barPressureBuffer[z];
    }
    // Add the new average values
    temp1Buffer[0] = temp1Ave / AveCounter;
    if (sdsSend == true)
    {
      pm2_5Buffer[0] = pm2_5Ave / PMAveCounter;
      pm10Buffer[0] = pm10Ave / PMAveCounter;
    }
    humidityBuffer[0] = humidityAve / AveCounter;
    barPressureBuffer[0] = barPressureAve / AveCounter;
    radiationBuffer[0] = radiationAve / radiationCounter;

    if (DEBUG_GRAPH == 1)
    {
      Serial.print("Graph Updated: ");
      Serial.print("Temp1 Ave: ");
      Serial.print(temp1Buffer[0]);
      Serial.print(" PM2.5 Ave: ");
      Serial.print(pm2_5Buffer[0]);
      Serial.print(" PM10 Ave: ");
      Serial.print(pm10Buffer[0]);
      Serial.print(" Humidity Ave: ");
      Serial.println(humidityBuffer[0]);
      Serial.print(" Radiation Ave: ");
      Serial.println(radiationBuffer[0]);
    }
    // reset the averages
    temp1Ave = 0;
    if (sdsSend == true)
    {
      pm2_5Ave = 0;
      pm10Ave = 0;
      PMAveCounter = 0;
    }
    humidityAve = 0;
    barPressureAve = 0;
    AveCounter = 0; // Reset the averages
    radiationAve = 0; // Reset the average
    radiationCounter = 0;

  }
  // ********** End graph update **************

  // Send the MQTT data - This is done at intervales: DATA_AVERAGE / NUMBER_SENSORS
  if (millis() >= (dataCounterTime + ((DATA_AVERAGE * 1000) / NUMBER_SENSORS)))
  {
    dataCounterTime = millis(); // Store new value for next time
    // Only do this when time is over the next update
    // *********** MQTT SEND VALUE(S) ***************
    // Want this to be non-blocking send data every alternate 2 seconds (200*10ms)

    // Ensure the connection to the MQTT server is alive (this will make the first
    // connection and automatically reconnect when disconnected).  See the MQTT_connect
    // function definition further below.
    MQTT_connect();

    switch (sensorMQTTSend)
    {
      case 0:
        // Send temperature
        if (DEBUG_MQTT == 1)
        {
          Serial.print(F("\nSending val "));
          Serial.print(temp1);
          Serial.print(F(" to airTemp feed..."));
        }
        if (! airTemp.publish(temp1)) {
          if (DEBUG_MQTT == 1)
          {
            Serial.println(F("Failed"));
          }
        } else {
          if (DEBUG_MQTT == 1)
          {
            Serial.println(F("OK!"));
          }
        }
        break;
      case 1:
        // Send PM2.5

        if (PMcounter < 2 && sdsSend == true)
        {
          if (DEBUG_MQTT == 1)
          {
            Serial.print(F("\nSending val "));
            Serial.print(pm2_5);
            Serial.print(F(" to airPM feed..."));
          }
          if (! airPM.publish(pm2_5)) {
            if (DEBUG_MQTT == 1)
            {
              Serial.println(F("Failed"));
            }
          } else {
            if (DEBUG_MQTT == 1)
            {
              Serial.println(F("OK!"));
            }
          }
          PMcounter++;
          if (PMcounter >= 2)
          {
            PMcounter = 0; // Reset the data send counter
            sdsSend = false;
          }
        }
        break;
      case 2:
        // Send PM10
        if (PMcounter < 2 && sdsSend == true) {
          if (DEBUG_MQTT == 1)
          {
            Serial.print(F("\nSending val "));
            Serial.print(pm10);
            Serial.print(F(" to airPM10 feed..."));
          }
          if (! airPM10.publish(pm10)) {
            if (DEBUG_MQTT == 1)
            {
              Serial.println(F("Failed"));
            }
          } else {
            if (DEBUG_MQTT == 1)
            {
              Serial.println(F("OK!"));
            }
          }
          PMcounter++;
          if (PMcounter >= 2)
          {
            PMcounter = 0; // Reset the data send counter
            sdsSend = false;
          }
        }
        break;
      case 3:
        // Send Humidity
        if (DEBUG_MQTT == 1)
        {
          Serial.print(F("\nSending val "));
          Serial.print(humidity);
          Serial.print(F(" to airHumidity feed..."));
        }
        if (! airHumidity.publish(humidity)) {
          if (DEBUG_MQTT == 1)
          {
            Serial.println(F("Failed"));
          }
        } else {
          if (DEBUG_MQTT == 1)
          {
            Serial.println(F("OK!"));
          }
        }
        break;
      case 4:
        // Send Pressure
        if (DEBUG_MQTT == 1)
        {
          Serial.print(F("\nSending val "));
          Serial.print(barPressure);
          Serial.print(F(" to airPressure feed..."));
        }
        if (! airPressure.publish(barPressure)) {
          if (DEBUG_MQTT == 1)
          {
            Serial.println(F("Failed"));
          }
        } else {
          if (DEBUG_MQTT == 1)
          {
            Serial.println(F("OK!"));
          }
        }
        break;
      case 5:
        // Send Radiation
        if (DEBUG_MQTT == 1)
        {
          Serial.print(F("\nSending val "));
          Serial.print(radiation);
          Serial.print(F(" to airRadiation feed..."));
        }
        if (! airRadiation.publish(radiation)) {
          if (DEBUG_MQTT == 1)
          {
            Serial.println(F("Failed"));
          }
        } else {
          if (DEBUG_MQTT == 1)
          {
            Serial.println(F("OK!"));
          }
        }
        break;
    }
    sensorMQTTSend++;
    if (sensorMQTTSend >= NUMBER_SENSORS)
    {
      sensorMQTTSend = 0; // Reset to zero
    }
  }
  // ******* END MQTT UPLOAD ********
  delay(10);  //Short delay to slow it all down!
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    mqttconnect = true;
    return;
  }
  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    mqttconnect = false;
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
  mqttconnect = true;
}

void updateScreen(int _mode, bool _wificonnect, bool _mqttconnect)
{
  // This routine draws the basic display with all features
  // Displays if WiFi connected
  // Displays if MQTT connected
  // Dsiplays mode (for debugging)
  u8g2.clearBuffer();
  u8g2.setFont(CBOLED_MESSAGE_FONT_8PT);  // choose a suitable font

  char _buffer[6];  // A holder for data to string conversions

  // Here we decide what to show depending upon the displayMode
  switch (displayMode) {
    case 1:
      // Show both temperatures as small values
      // Show both temperatures as small values
      u8g2.setCursor(0, 10);
      u8g2.print(F("Environment Unit"));
      u8g2.setCursor(0, 25);
      u8g2.print(F("T:"));
      u8g2.setCursor(38, 25);
      u8g2.print(checkTempRange(temp1, -100.0, 120.0));
      u8g2.setCursor(0, 35);
      u8g2.print(F("PM25:"));
      u8g2.setCursor(38, 35);
      u8g2.print((int)pm2_5);
      u8g2.setCursor(0, 45);
      u8g2.print(F("PM10:"));
      u8g2.setCursor(38, 45);
      u8g2.print((int)pm10);

      u8g2.setCursor(70, 25);
      u8g2.print(F("HUM:"));
      u8g2.setCursor(110, 25);
      u8g2.print((int)humidity);
      u8g2.setCursor(70, 35);
      u8g2.print(F("mmHg:"));
      u8g2.setCursor(110, 35);
      // BarPressure is in Pa - divide by 133 to give approx mm of mercury (mmHg)
      u8g2.print((int)(barPressure / 133));
      u8g2.setCursor(70, 45);
      u8g2.print(F("Rad:"));
      u8g2.setCursor(110, 45);
      // BarPressure is in Pa - divide by 133 to give approx mm of mercury (mmHg)
      u8g2.print((int)(radiation));

      pixels.clear(); // Set all pixel colors to 'off'
      pixels.show();   // Send the updated pixel colors to the hardware.
      break;
    case 2:
      u8g2.setCursor(0, 10);
      u8g2.print(F("MIN:"));
      // String checkTempRange(float _temp, float _min, float _max)
      u8g2.setCursor(32, 10);
      u8g2.print(checkTempRange(temp1Min, -100.0, 120.0));
      u8g2.setCursor(64, 10);
      u8g2.print(F("MAX:"));
      u8g2.setCursor(96, 10);
      u8g2.print(checkTempRange(temp1Max, -100.0, 120.0));
      // Draw the main temp in BIG in the middle
      u8g2.setCursor(0, 36);
      u8g2.print(F("T air:"));
      // Want to adjust the font here:
      u8g2.setFont(CBOLED_MESSAGE_FONT_24PT);  // choose a suitable font
      u8g2.setCursor(38, 50);
      u8g2.print(checkTempRange(temp1, -100.0, 120.0));
      u8g2.setFont(CBOLED_MESSAGE_FONT_8PT);  // Font back to normal!
      checkLEDs(temp1, tempHigh, tempLow);
      break;

    case 3:
      // Show temp 1 as a bar chart over time
      u8g2.setCursor(0, 10);
      u8g2.print(F("T air:"));
      u8g2.setCursor(64, 10);
      u8g2.print(checkTempRange(temp1, -100.0, 120.0));
      // Want to draw 120 lines from
      // startx, staryy graphHeight, graphHeightMaxValue
      for (int n = 0; n < sizeOfBuffer; n++)
      {
        if (temp1Buffer[n] <= graphHeightMaxValue && temp1Buffer[n] >= graphHeightMinValue)
        {
          graphCalculatedY = (starty - (((temp1Buffer[n] - graphHeightMinValue) / (graphHeightMaxValue - graphHeightMinValue)) * graphHeight ));
        }
        else if (temp1Buffer[n] > graphHeightMaxValue)
        {
          graphCalculatedY = (starty - (graphHeight));
        }
        else if (temp1Buffer[n] < graphHeightMinValue)
        {
          graphCalculatedY = starty;
        }
        u8g2.drawLine(startx + n, starty, startx + n, (int)graphCalculatedY);
      }
      checkLEDs(temp1, tempHigh, tempLow);
      break;
    case 4:
      u8g2.setCursor(0, 10);
      u8g2.print(F("MIN:"));
      u8g2.setCursor(32, 10);
      u8g2.print(pm2_5Min);
      u8g2.setCursor(64, 10);
      u8g2.print(F("MAX:"));
      u8g2.setCursor(96, 10);
      u8g2.print(pm2_5Max);
      // Draw the main temp in BIG in the middle
      u8g2.setCursor(0, 36);
      u8g2.print(F("PM2.5:"));
      // Want to adjust the font here:
      u8g2.setFont(CBOLED_MESSAGE_FONT_24PT);  // choose a suitable font
      u8g2.setCursor(38, 50);
      dtostrf(pm2_5, 3, 0, _buffer);  // 0 DP
      u8g2.print(_buffer);
      u8g2.setFont(CBOLED_MESSAGE_FONT_8PT);  // Font back to normal!
      checkLEDs(pm2_5, 0, 50);
      break;
    case 5:
      // Show temp 1 as a bar chart over time
      u8g2.setCursor(0, 10);
      u8g2.print(F("PM2.5:"));
      u8g2.setCursor(64, 10);
      u8g2.print(pm2_5);
      // Want to draw 120 lines from
      // startx, staryy graphHeight, graphHeightMaxValue
      for (int n = 0; n < sizeOfBuffer; n++)
      {
        if (pm2_5Buffer[n] <= graphPM25HeightMaxValue && pm2_5Buffer[n] >= graphPM25HeightMinValue)
        {
          graphCalculatedY = (starty - (((pm2_5Buffer[n] - graphPM25HeightMinValue) / (graphPM25HeightMaxValue - graphPM25HeightMinValue)) * graphHeight ));
        }
        else if (pm2_5Buffer[n] > graphPM25HeightMaxValue)
        {
          graphCalculatedY = starty - (graphHeight);
        }
        else if (pm2_5Buffer[n] < graphPM25HeightMinValue)
        {
          graphCalculatedY = starty;
        }
        u8g2.drawLine(startx + n, starty, startx + n, (int)graphCalculatedY);
      }
      checkLEDs(pm2_5, 0, 50);
      break;
    case 6:
      u8g2.setCursor(0, 10);
      u8g2.print(F("MIN:"));
      u8g2.setCursor(32, 10);
      u8g2.print(pm10Min);
      u8g2.setCursor(64, 10);
      u8g2.print(F("MAX:"));
      u8g2.setCursor(96, 10);
      u8g2.print(pm10Max);
      // Draw the main temp in BIG in the middle
      u8g2.setCursor(0, 36);
      u8g2.print(F("PM10:"));
      // Want to adjust the font here:
      u8g2.setFont(CBOLED_MESSAGE_FONT_24PT);  // choose a suitable font
      u8g2.setCursor(38, 50);
      dtostrf(pm10, 3, 0, _buffer);  // 0 DP
      u8g2.print(_buffer);
      u8g2.setFont(CBOLED_MESSAGE_FONT_8PT);  // Font back to normal!
      checkLEDs(pm10, 0, 100);
      break;
    case 7:
      // Show temp 1 as a bar chart over time
      u8g2.setCursor(0, 10);
      u8g2.print(F("PM10:"));
      u8g2.setCursor(64, 10);
      u8g2.print(pm10);
      // Want to draw 120 lines from
      // startx, staryy graphHeight, graphHeightMaxValue
      for (int n = 0; n < sizeOfBuffer; n++)
      {
        if (pm10Buffer[n] <= graphPM10HeightMaxValue && pm10Buffer[n] >= graphPM10HeightMinValue)
        {
          graphCalculatedY = (starty - (((pm10Buffer[n] - graphPM10HeightMinValue) / (graphPM10HeightMaxValue - graphPM10HeightMinValue)) * graphHeight ));
        }
        else if (pm2_5Buffer[n] > graphPM10HeightMaxValue)
        {
          graphCalculatedY = starty - (graphHeight);
        }
        else if (pm2_5Buffer[n] < graphPM10HeightMinValue)
        {
          graphCalculatedY = starty;
        }
        u8g2.drawLine(startx + n, starty, startx + n, (int)graphCalculatedY);
      }
      checkLEDs(pm10, 0, 100);
      break;
    case 8:
      u8g2.setCursor(0, 10);
      u8g2.print(F("MIN:"));
      u8g2.setCursor(32, 10);
      u8g2.print(humidityMin);
      u8g2.setCursor(64, 10);
      u8g2.print(F("MAX:"));
      u8g2.setCursor(96, 10);
      u8g2.print(humidityMax);
      // Draw the main temp in BIG in the middle
      u8g2.setCursor(0, 36);
      u8g2.print(F("Humidity:"));
      // Want to adjust the font here:
      u8g2.setFont(CBOLED_MESSAGE_FONT_24PT);  // choose a suitable font
      u8g2.setCursor(38, 50);
      dtostrf(humidity, 3, 0, _buffer);  // 0 DP
      u8g2.print(_buffer);
      u8g2.setFont(CBOLED_MESSAGE_FONT_8PT);  // Font back to normal!
      //checkLEDs(temp2, tempHigh, tempLow);
      pixels.clear(); // Set all pixel colors to 'off'
      pixels.show();   // Send the updated pixel colors to the hardware.
      break;
    case 9:
      // Show temp 1 as a bar chart over time
      u8g2.setCursor(0, 10);
      u8g2.print(F("Humidity:"));
      u8g2.setCursor(64, 10);
      u8g2.print(humidity);
      // Want to draw 120 lines from
      // startx, staryy graphHeight, graphHeightMaxValue
      for (int n = 0; n < sizeOfBuffer; n++)
      {
        if (humidityBuffer[n] <= graphHumidityHeightMaxValue && humidityBuffer[n] >= graphHumidityHeightMinValue)
        {
          graphCalculatedY = (starty - (((humidityBuffer[n] - graphHumidityHeightMinValue) / (graphHumidityHeightMaxValue - graphHumidityHeightMinValue)) * graphHeight ));
        }
        else if (pm2_5Buffer[n] > graphHumidityHeightMaxValue)
        {
          graphCalculatedY = starty - (graphHeight);
        }
        else if (pm2_5Buffer[n] < graphHumidityHeightMinValue)
        {
          graphCalculatedY = starty;
        }
        u8g2.drawLine(startx + n, starty, startx + n, (int)graphCalculatedY);
      }
      //checkLEDs(temp2, tempHigh, tempLow);
      pixels.clear(); // Set all pixel colors to 'off'
      pixels.show();   // Send the updated pixel colors to the hardware.
      break;
    case 10:
      // Draw the main temp in BIG in the middle
      u8g2.setCursor(0, 10);
      u8g2.print(F("Pressure:"));
      // Want to adjust the font here:
      u8g2.setFont(CBOLED_MESSAGE_FONT_24PT);  // choose a suitable font
      u8g2.setCursor(2, 50);
      dtostrf(barPressure, 6, 0, _buffer);  // 0 DP
      u8g2.print(_buffer);
      u8g2.setFont(CBOLED_MESSAGE_FONT_8PT);  // Font back to normal!
      pixels.clear(); // Set all pixel colors to 'off'
      pixels.show();   // Send the updated pixel colors to the hardware.
      break;
    case 11:
      // Show temp 1 as a bar chart over time
      u8g2.setCursor(0, 10);
      u8g2.print(F("Pressure:"));
      u8g2.setCursor(64, 10);
      u8g2.print(barPressure);
      // Want to draw 120 lines from
      // startx, staryy graphHeight, graphHeightMaxValue
      for (int n = 0; n < sizeOfBuffer; n++)
      {
        if (barPressureBuffer[n] <= graphbarPressureHeightMaxValue && barPressureBuffer[n] >= graphbarPressureHeightMinValue)
        {
          graphCalculatedY = (starty - (((barPressureBuffer[n] - graphbarPressureHeightMinValue) / (graphbarPressureHeightMaxValue - graphbarPressureHeightMinValue)) * graphHeight ));
        }
        else if (barPressureBuffer[n] > graphbarPressureHeightMaxValue)
        {
          graphCalculatedY = starty - (graphHeight);
        }
        else if (barPressureBuffer[n] < graphbarPressureHeightMinValue)
        {
          graphCalculatedY = starty;
        }
        u8g2.drawLine(startx + n, starty, startx + n, (int)graphCalculatedY);
      }
      //checkLEDs(temp2, tempHigh, tempLow);
      pixels.clear(); // Set all pixel colors to 'off'
      pixels.show();   // Send the updated pixel colors to the hardware.
      break;
    case 12:
      u8g2.setCursor(0, 10);
      u8g2.print(F("MIN:"));
      u8g2.setCursor(32, 10);
      u8g2.print(radiationMin);
      u8g2.setCursor(64, 10);
      u8g2.print(F("MAX:"));
      u8g2.setCursor(96, 10);
      u8g2.print(radiationMax);

      // Draw the main temp in BIG in the middle
      u8g2.setCursor(0, 36);
      u8g2.print(F("Radiation:"));
      // Want to adjust the font here:
      u8g2.setFont(CBOLED_MESSAGE_FONT_24PT);  // choose a suitable font
      u8g2.setCursor(38, 50);
      dtostrf(radiation, 3, 0, _buffer);  // 0 DP
      u8g2.print(_buffer);
      u8g2.setFont(CBOLED_MESSAGE_FONT_8PT);  // Font back to normal!
      checkLEDs(radiation, 100, 0);
      break;
    case 13:
      // Show temp 1 as a bar chart over time
      u8g2.setCursor(0, 10);
      u8g2.print(F("Radiation:"));
      u8g2.setCursor(64, 10);
      u8g2.print(radiation);
      // Want to draw 120 lines from
      // startx, staryy graphHeight, graphHeightMaxValue
      for (int n = 0; n < sizeOfBuffer; n++)
      {
        if (radiationBuffer[n] <= graphRadiationHeightMaxValue && barPressureBuffer[n] >= graphRadiationHeightMinValue)
        {
          graphCalculatedY = (starty - (((radiationBuffer[n] - graphRadiationHeightMinValue) / (graphRadiationHeightMaxValue - graphRadiationHeightMinValue)) * graphHeight ));
        }
        else if (radiationBuffer[n] > graphRadiationHeightMaxValue)
        {
          graphCalculatedY = starty - (graphHeight);
        }
        else if (radiationBuffer[n] < graphRadiationHeightMinValue)
        {
          graphCalculatedY = starty;
        }
        u8g2.drawLine(startx + n, starty, startx + n, (int)graphCalculatedY);
      }
      checkLEDs(radiation, 100, 0);
      break;
    case 99:
      // This is the case when the EEPROM has been saved
      // Displays this screen for a bit!
      u8g2.setCursor(0, 10);
      u8g2.print(F("MODE SAVED!"));
      displayMode = EEPROM.read(10);  // After showing - update to mode
      break;
    case 100:
      // This is the case when the EEPROM has been saved
      // Displays this screen for a bit!
      u8g2.setCursor(0, 10);
      u8g2.print(F("START UP"));
      break;
  }

  // This section draws the bit at the botoom (always there)
  if (_wificonnect == true)
  {
    u8g2.setCursor(0, 64);
    u8g2.print(F("Wifi OK"));
  }
  else
  {
    u8g2.setCursor(0, 64);
    u8g2.print(F("       "));
  }
  if (_mqttconnect == true)
  {
    u8g2.setCursor(64, 64);
    u8g2.print(F("MQTT OK"));
  }
  else
  {
    u8g2.setCursor(64, 64);
    u8g2.print(F("       "));
  }

  //  // Decomment this to show the displayMode for debugging
  //  dtostrf(_mode, 7, 0, _buffer);
  //  u8g2.setCursor(100, 64);
  //  u8g2.print(_buffer);

  u8g2.sendBuffer();  // Write all the display data to the screen
}

void checkLEDs(float _temp, float _tempHigh, float _tempLow)
{
  // This lights the LEDs in different colours depending upon high/low setpoints
  if (_temp > _tempHigh)
  {
    // Show red as too warm
    for (int i = 0; i < RGBLED_COUNT; i++)
    {
      pixels.setPixelColor(i, 255, 0, 0);
    }
  }
  else   if (_temp < _tempLow)
  {
    // Show Blue as too cold
    for (int i = 0; i < RGBLED_COUNT; i++)
    {
      pixels.setPixelColor(i, 0, 0, 255);
    }
  }
  else
  {
    //  Show green as Just right
    for (int i = 0; i < RGBLED_COUNT; i++)
    {
      pixels.setPixelColor(i, 0, 255, 0);
    }
  }
  pixels.setBrightness(RGBLED_BRIGHTNESS);
  pixels.show();
}



// ****** ENCODER & BUTTON FUNCTIONS *****************
// on change of encoder
void rotate(ESPRotary & r) {
  //RotaryValue = r.getPosition();
  //  Serial.println(r.directionToString(r.getDirection()));
  //  Serial.println(r.getPosition());
  if (DEBUG_ENCODER == 1)
  {
    Serial.println(r.directionToString(r.getDirection()));
    Serial.print("Position: ");
    Serial.println(r.getPosition());
  }

  if (r.directionToString(r.getDirection()) == "RIGHT")
  {
    displayMode++;
    if (displayMode > maxDisplayMode)
    {
      displayMode = 1;
    }
  }
  else if (r.directionToString(r.getDirection()) == "LEFT")
  {
    displayMode--;
    if (displayMode <= 0)
    {
      displayMode = maxDisplayMode;
    }
  }
  updateScreen(displayMode, wificonnect, mqttconnect);
}

// ****** Check if temperature value is within bounds *******

String checkTempRange(float _temp, float _min, float _max)
{
  char _localBuffer[4];
  String _string;
  // Check temp and return a string of value if OK
  // Return string of "NA" if not
  if (_temp > _max || _temp < _min)
  {
    _string = "NA  ";
    // Sensor TOO high - probably not connected
    _string.toCharArray(_localBuffer, 4);
  }
  else
  {
    dtostrf(_temp, 4, 1, _localBuffer);
  }
  return (_localBuffer);
}

// long click of button
void click(Button2 & btn) {
  //Store starting displayMode to EEPROM with long press
  EEPROM.write(10, displayMode);  // this writes a good value to it
  EEPROM.commit();
  Serial.println("MODE Saved");
  displayMode = 99; // Show the 'saved' screen
  updateScreen(displayMode, wificonnect, mqttconnect);
}
