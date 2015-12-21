/* R-IoT :
 Texas Instrument CC3200 Internet of Things / Sensor hub / Dev Platform
 80 MHz 32 Bit ARM MCU + Wifi stack / modem
 
 IRCAM - Emmanuel FLETY - Music Bricks
 
 Battery voltage : protection voltage divider of 0.364. ADC vRef (and max input) = 1.7V (ish)
 Battery reading in volt = [analogRead() * 1.7] / (4096 * 0.364)
 or, simplified to 
 Battery reading in volt = analogRead() / 877.
 
 Rev History :
 
 1.5 : adding a AP style connection to allow streaming to multiple computers / devices
 
 1.4 :  loads of fixing in the calibration process for the absolute angles (madgwick)
 and webserver
 
 */

#include <stdio.h>
#include <strings.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include <Wire.h>

// Handles the file system of the FLASH, to store parameters
#include <SLFS.h>

#include "common.h"
#include "LSM9DS0.h"
#include "osc.h"
#include "web.h"

/////////////////////////////////////////////////////////////
// DEFAULT parameters
// your network name also called SSID
byte mac[6];
const char TheSSID[] = "riot";
const uint16_t TheDestPort = DEFAULT_UDP_PORT;
const uint8_t TheLocalIP[] = {
  192,168,1,40};
const uint8_t TheSubnetMask[] = {
  255,255,255,0};
const uint8_t TheGatewayIP[] = {
  192,168,1,1};
const uint8_t TheDestIP[] = {
  192,168,1,100};
const unsigned long TheSampleRate = DEFAULT_SAMPLE_RATE;
const uint8_t TheID = 0; 

/////////////////////////////////////////////////////////////
// Global vars
WiFiServer server(80);
WiFiClient client;
WiFiClient clientStreaming[MAX_CLIENTS];
byte APorStation = STATION_MODE;
char ssid[32];
char ssidAP[32];
char password[32] = "12345678";
IPAddress LocalIP;
IPAddress APIP;
IPAddress SubnetMask;
IPAddress GatewayIP;
IPAddress DestIP;
uint16_t DestPort;
uint8_t ModuleID;
unsigned long SampleRate;
boolean UseDHCP = true;
boolean UseSecurity = false;
int status = WL_IDLE_STATUS;
int statusAP = false;
int PacketStatus;
boolean ConfigurationMode = false;
boolean AcceptOSC = false;
byte PageToDisplay = CONFIG_WEB_PAGE;
unsigned int ConfigModeCounter = 0;
int TempInt = 0;

char packetBuffer[255]; //buffer to hold incoming packet
WiFiUDP UdpPacket;
WiFiUDP ConfigPacket;
OscBuffer RawSensors;
OscBuffer Quaternions;
OscBuffer EulerAngles;
OscBuffer AnalogInputs;

unsigned long ElapsedTime = 0;
unsigned long ElapsedTime2 = 0;

////////////////////////////////////////////////////////////
// Sensor storage
short unsigned int BatteryVoltage = 0, SwitchState;
Word AccelerationX, AccelerationY, AccelerationZ;
Word GyroscopeX, GyroscopeY, GyroscopeZ;
Word MagnetometerX, MagnetometerY, MagnetometerZ;
Word Temperature;

short unsigned int AnalogInput1, AnalogInput2;

// Defines whether you want the "raw" value with the stored offset or not
byte SendCalibrated = true;
byte SensorOrientation = BOTTOM;  // wifi chip & switches facing up
//byte SensorOrientation = TOP;  // sensor facing up

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Absolute angle (madgwick)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define PI 3.14159265358979323846264338327950
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
// Beta is called the rate of convergence of the filter. Higher value lead to a noisy output but fast response.
// The best is to first set zeta (gyro bias drift) to zero and adjust beta to get the proper convergence behavior,
// then adjust zeta to a suit the gyro bias drift.

//#define zeta  0.1f // zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define ZETA_DEFAULT  0.0f
#define BETA_DEFAULT 0.4f    // Much faster - noisier
//#define beta 0.041f    // 10 seconds or more to have the quaternion converge to the right orientation - super smooth

float zeta = ZETA_DEFAULT;
float beta = BETA_DEFAULT;

float pitch, yaw, roll, heading;
float deltat = 0.005f;        // integration interval for both filter schemes - 5ms by default

int gyro_bias[3] = { 0, 0, 0};
int accel_bias[3] = { 0, 0, 0};
int mag_bias[3] = { 0, 0, 0};
int bias_samples = 32;

float abias[3] = { 0., 0., 0.};
float gbias[3] = { 0., 0., 0.};
float mbias[3] = { 0., 0., 0.};

float gRes, aRes, mRes;		// Resolution = Sensor range / 2^15
float a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z; // variables to hold latest sensor data values
float mag_nobias[3];
float mag_cal[3];  // calibrated version of the mag sensor data

float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error
float b_x = 1, b_z = 0; // reference direction of flux in earth frame
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions

/////////////////////////////////////////////////////////////////////////////
// Magnetic Sensors calibration using a rotation / transformation matrix
float rotation_matrix[3][3] = 
{
  { 1.0, 0.0, 0.0 },
  { 0.0, 1.0, 0.0 },
  { 0.0, 0.0, 1.0 } 
};

float identity_matrix[3][3] = 
{
  { 1.0, 0.0, 0.0 },
  { 0.0, 1.0, 0.0 },
  { 0.0, 0.0, 1.0 } 
};

float rotation_matrix_result[3][3];
byte CalibrationStage = START;

// MAG Vectors for all cardinal positions (3 items per array, X, Y, Z)
// Used for elipsoid fitting
float Xplus0[3];
float Xplus180[3];
float Xminus0[3];
float Xminus180[3];

float Yplus0[3];
float Yplus180[3];
float Yminus0[3];
float Yminus180[3];

float Zplus0[3];
float Zplus180[3];
float Zminus0[3];
float Zminus180[3];


/////////////////////////////////////////////////////////////////
// Serial port message / buffers / temporary strings
char SerialBuffer[MAX_SERIAL];
unsigned char SerialIndex = 0;
boolean FlagSerial = FALSE;
char StringBuffer[MAX_STRING];


// To get printf to work, we redirect STDOUT and the myWrite functions
ssize_t myWrite(void *cookie, const char *buf, size_t n)
{
  return Serial.write((uint8_t*)buf, n);
}

cookie_io_functions_t myVectors = { 
  0, myWrite, 0, 0 };

void setup() {
  // Basic I/Os
  pinMode(POWER_LED, OUTPUT);
  pinMode(SWITCH_INPUT, INPUT_PULLUP);

  // POWER On indicator  
  digitalWrite(POWER_LED, HIGH);

  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  Serial.println(VERSION_DATE);

  // Needed to have printf working
  stdout = fopencookie((void *)0, "w", myVectors);
  setlinebuf(stdout);

  digitalWrite(POWER_LED, HIGH);

  // Check if we are going in configuration mode
  // 2-3 second pressing on the switch during boot
  while(!digitalRead(SWITCH_INPUT))
  {
    //Serial.println("switch pressed");
    delay(20);
    TempInt++;
    if(digitalRead(POWER_LED))
      digitalWrite(POWER_LED, LOW);
    else
      digitalWrite(POWER_LED, HIGH);

    if(TempInt > WEB_SERVER_DELAY)
    {
      ConfigurationMode = true;
      Serial.println("Configuration / Web Server Mode");
      digitalWrite(POWER_LED, HIGH);
      break;
    }
  }

  // Init motion sensor
  Serial.println("Init Motion Sensor");
  // Start SPI with defaults
  SPI.begin();
  // SPI settings
  // 16 MHz max bit rate, clock divider 1:2 => 8 MHZ SPI clock
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  InitLSM9DS0();

  // Scaling to obtain gs and deg/s 	
  // Must match the init settings of the LSM9DS0
  // Beware, absolute dynamic range of each sensor isn't equivalent.
  // Accelerometers are using the whole dynamic range, gyro / mag aren't
  gRes = 2000.0 / 28571.4; 	// +- 2000 deg/s
  aRes = 8.0 / 32768.0;         // +- 8g
  mRes = 2.0 / 25000.0;         // +- 2 gauss

  // Starts the file system
  Serial.println("Loading up params");

  SerFlash.begin();

  // Retrieve saved params in FLASH using the file system
  LoadParams();

  // Finalize the bias unit conversion
  for(int i = 0 ; i < 3 ; i++)
  {
    gbias[i] = gRes * (float)gyro_bias[i];
    abias[i] = aRes * (float)accel_bias[i];
    mbias[i] = mRes * (float)mag_bias[i];
  }

  if(!ConfigurationMode)
  {
    digitalWrite(POWER_LED, LOW);
    
    if(APorStation == STATION_MODE)
    {
      // attempt to connect to Wifi network:
      Serial.print("R-IoT connecting to: ");
      // print the network name (SSID);
      Serial.println(ssid); 
      Connect(); 
    }
    else  // AP mode
    {
      // attempt to connect to Wifi network:
      Serial.print("R-IoT creates network: ");
      // print the network name (SSID);
      Serial.println(ssid); 
      
      // Creates the AP & config
      APIP = IPAddress(TheGatewayIP);
      WiFi.config(APIP);
      if(!UseSecurity)
        WiFi.beginNetwork((char *)ssid);
      else
        WiFi.beginNetwork((char *)ssid, (char*)password);
    }
    
    WiFi.macAddress(mac);
    
    // Prep the UDP packet
    UdpPacket.begin(DestPort);
    UdpPacket.beginPacket(DestIP, DestPort);
    
    // Open the service port to talk to the module (config, calibration)
    ConfigPacket.begin(DEFAULT_UDP_SERVICE_PORT);

    // Prepare the OSC message structure
    sprintf(StringBuffer, "/%u/raw\0",ModuleID);
    PrepareOSC(&RawSensors, StringBuffer, 'i', 12);    // 11 integers + temperature
    sprintf(StringBuffer, "/%u/quat\0",ModuleID);
    PrepareOSC(&Quaternions, StringBuffer, 'f', 4);    // 4 floats
    sprintf(StringBuffer, "/%u/euler\0",ModuleID);
    PrepareOSC(&EulerAngles, StringBuffer, 'f', 4);    // 4 floats (yaw, pitch, roll, heading)
    sprintf(StringBuffer, "/%u/analog\0",ModuleID);
    PrepareOSC(&AnalogInputs, StringBuffer, 'i', 2);    // 2 int    

    // Starts the I2C bus if needed
    //Wire.begin();
    
  } // END OF IF NORMAL (!CONFIG) MODE

  // If in configuration mode we setup a webserver for configuring the unit
  // The module becomes an AP with DHCP
  else
  {
    APIP = IPAddress(TheGatewayIP);

    WiFi.config(APIP);
    randomSeed(analogRead(BATT_MONITORING));
    sprintf(ssidAP, "RIOT-%04x\0",random(16000));
    Serial.print("Setting up Access Point named: ");
    Serial.println(ssidAP);
    WiFi.beginNetwork((char *)ssidAP);
    WiFi.macAddress(mac);
  }

  ElapsedTime = millis();
  ElapsedTime2 = millis();
}



void loop() {

  if(!ConfigurationMode)
  {
    if((millis() - ElapsedTime2) > 300) // Perform the check not too often
    {
      ElapsedTime2 = millis();
      if(APorStation == STATION_MODE)
      {
        int CurrentStatus = WiFi.status();
        if(CurrentStatus != WL_CONNECTED)
        {
          //printf("wifi status = %d\n", WiFi.status());
          // print dots while we wait to connect and blink the power led
          Serial.print(".");
          if(digitalRead(POWER_LED))
            digitalWrite(POWER_LED, LOW);
          else
            digitalWrite(POWER_LED, HIGH);
        }
        // Newly connected to the network, locks until DHCP answers
        // if enabled 
        if((status != WL_CONNECTED) && (WiFi.status() == WL_CONNECTED))
        {
          status = WiFi.status();
          digitalWrite(POWER_LED, HIGH);
          Serial.println("\nConnected to the network");  

          while ((WiFi.localIP() == INADDR_NONE))
          {
            // print dots while we wait for an ip addresss
            Serial.print(".");
            delay(300);  
          }
          // you're connected now, so print out the status  
          printCurrentNet();
          printWifiData();  
        }
        
        // Disconnected from the network, try to reconnect
        if((status == WL_CONNECTED) && (WiFi.status() != WL_CONNECTED))
        {
          Serial.println("Network Lost, trying to reconnect");
          status = WiFi.status();
          // Debug
          //Serial.print("Current Status = ");
          //Serial.println(status);
        }
      } // End of Station mode connection
      else  // AP mode
      {      
        if(WiFi.localIP() == INADDR_NONE)   // Indicates AP isn't ready yet
        {        
          // print dots while we wait to connect and blink the power led
          Serial.print(".");
          if(digitalRead(POWER_LED))
            digitalWrite(POWER_LED, LOW);
          else
            digitalWrite(POWER_LED, HIGH);
        }  
        else if (!statusAP)
        {
          statusAP = true;
          digitalWrite(POWER_LED, HIGH);
          Serial.println("AP active.");
          printCurrentNet();
          printWifiData();
        }
      } // End of AP mode connection
    } // end of IF(elapsed time)

    // Checking if an incoming UDP packet is theres is blocking with a minimum
    // check duration of 10 ms. We don't check unless we're in configuration mode
    if(AcceptOSC)
    {
      // Parses incoming OSC messages
      digitalWrite(POWER_LED, HIGH);
      int packetSize = ConfigPacket.parsePacket();
      if (packetSize)
      {
        // Debug Info
         //Serial.print("Received packet of size ");
         //Serial.println(packetSize);
         //Serial.print("From ");
         //IPAddress remoteIp = ConfigPacket.remoteIP();
         //Serial.print(remoteIp);
         //Serial.print(", port ");
         //Serial.println(ConfigPacket.remotePort());

        // read the packet into packetBufffer
        int Index = 0;
        int len = ConfigPacket.read(packetBuffer, 255);
        if (len > 0) packetBuffer[len] = 0;          // Add a terminator in the buffer
        //Serial.println("Contents:");
        //Serial.println(packetBuffer);
        //Serial.println("Packet Len:");
        //Serial.println(len);
        // Actual parsing
        // Checks that's for the proper ID / module
        sprintf(StringBuffer, "/%u/\0",ModuleID);
        if(!strncmp(packetBuffer, StringBuffer, strlen(StringBuffer)))
        {  // that's for us
          char *pUDP = packetBuffer + strlen(StringBuffer);
          if(!strncmp(pUDP, "calibrate", 9))
          {
            Calibrate();
          }
          // add here other keywords like changing the sample rate and saving 
          // data => see parse serial
          else if(!strncmp(pUDP, "rate", 4))
          {
            Serial.println("Sample Rate update");      
            for(int i=0 ; i < len ; i++)
            {
                Serial.print(packetBuffer[i]);
                Serial.print(" ");
            }      
            //SampleRate = atoi(&(StringBuffer[Index]));
            // needs parameter value in the OSC message, needs more work
          }

        }
      }
      digitalWrite(POWER_LED, LOW);
    }

    if((millis() - ElapsedTime >= SampleRate) && !ConfigurationMode &&
    (((WiFi.status()==WL_CONNECTED) && (APorStation==STATION_MODE)) ||
    (statusAP && (APorStation==AP_MODE))))
    {       
      ElapsedTime = millis();
      digitalWrite(POWER_LED, HIGH);
      // read the battery status
      BatteryVoltage = analogRead(BATT_MONITORING);
      SwitchState = digitalRead(SWITCH_INPUT);
      
      // Comment those 2 if you don't need the analog inputs to be exported by OSC
      AnalogInput1 = analogRead(ANALOG_INPUT1);
      AnalogInput2 = analogRead(ANALOG_INPUT2);

      if(!SwitchState)
      { 
        ConfigModeCounter++;
        if(ConfigModeCounter > 1000)
        {
          ConfigModeCounter = 0;
          if(AcceptOSC)
          {
            AcceptOSC = false;
            Serial.println("Leaving OSC receive mode");
          }
          else
          {
            AcceptOSC = true;
            Serial.println("Entering OSC receive mode");
          }
        }
      }
      else
        ConfigModeCounter = 0;

      // Debug
      //Serial.print("Battery voltage=");
      //Serial.println(BatteryVoltage);
      ReadAccel();
      ReadGyro();
      ReadMagneto();
      ReadTemperature();
    
      g_x = (gRes * (float)GyroscopeX.Value) - gbias[0];   // Convert to degrees per seconds, remove gyro biases
      g_y = (gRes * (float)GyroscopeY.Value) - gbias[1];
      
      // Small noise gate on the gyro Z to reduce integration drift on YAW
      // Gyro offset drits with temperature, but it's so far hard to calibrate.
      // So little static offset is removed to avoid the madgwick algorithm to "think" the 
      // gyro is actually spinning. The noise gate level should be kept low to keep the
      // integration working ok and give proper angle after long rotations.
      if(abs(GyroscopeZ.Value - gyro_bias[2]) < GYRO_NOISEGATE)
        g_z = 0.0f;
      else
        g_z = (gRes * (float)GyroscopeZ.Value) - gbias[2];

      a_x = (aRes * (float)AccelerationX.Value) - abias[0];   // Convert to g's, remove accelerometer biases
      a_y = (aRes * (float)AccelerationY.Value) - abias[1];
      a_z = (aRes * (float)AccelerationZ.Value) - abias[2];

      m_x = (mRes * (float)MagnetometerX.Value) - mbias[0];     // Convert to Gauss and correct for calibration
      m_y = (mRes * (float)MagnetometerY.Value) - mbias[1];
      m_z = (mRes * (float)MagnetometerZ.Value) - mbias[2];   

      // Mag calibration thru transformation matrix
      mag_nobias[0] = m_x;
      mag_nobias[1] = m_y;
      mag_nobias[2] = m_z;
      CalibrateMag(mag_cal, mag_nobias);
  
      // OLD with non calibrated MAG - NOT USED ANYMORE
      //MadgwickQuaternionUpdate(-a_x, -a_y, -a_z, g_x*PI/180.0f, g_y*PI/180.0f, g_z*PI/180.0f, -m_x, -m_y, m_z);
    
      ////////////////////////////////////////////////////////////////////////////////////
      // Note regarding the sensor orientation & angles :
      // We alter the sensor sign in order to "redefine gravity" and axis so that it behaves
      // the same whether the sensor is up or down. However, for the heading computation and
      // correction against angles, the "real" sensor orientation and pitch / roll proper 
      // signing must be used. We therefore do a double signe inversion when the sensor 
      // is on the bottom. [looks crappy but works and for good reasons]
    
      // Default Orientation, sensor on top (board reversed)
      if(SensorOrientation == TOP)
      {
        MadgwickQuaternionUpdate(a_x, a_y, a_z, g_x*PI/180.0f, g_y*PI/180.0f, g_z*PI/180.0f, mag_cal[0], mag_cal[1], -mag_cal[2]);
        yaw   = atan2(2.0f * (SEq_2 * SEq_3 + SEq_1 * SEq_4), SEq_1 * SEq_1 + SEq_2 * SEq_2 - SEq_3 * SEq_3 - SEq_4 * SEq_4);   
        pitch = -asin(2.0f * ((SEq_2 * SEq_4) - (SEq_1 * SEq_3)));
        roll  = atan2(2.0f * (SEq_1 * SEq_2 + SEq_3 * SEq_4), (SEq_1 * SEq_1) - (SEq_2 * SEq_2) - (SEq_3 * SEq_3) + (SEq_4 * SEq_4));
      }
      else  // WiFi chip and switches facing up
      {
        MadgwickQuaternionUpdate(-a_x, -a_y, -a_z, g_x*PI/180.0f, g_y*PI/180.0f, g_z*PI/180.0f, -mag_cal[0], -mag_cal[1], mag_cal[2]);     
        yaw   = atan2(2.0f * (SEq_2 * SEq_3 + SEq_1 * SEq_4), SEq_1 * SEq_1 + SEq_2 * SEq_2 - SEq_3 * SEq_3 - SEq_4 * SEq_4);   
        // Small trigo cheat as the module is upside down so north is south and conversely, and axis orientation are swaped.
        yaw = (yaw - PI);
        if(yaw < -PI)
          yaw += 2 * PI;        
        pitch = asin(2.0f * ((SEq_2 * SEq_4) - (SEq_1 * SEq_3))); // pitch is inverted
        roll  = atan2(2.0f * (SEq_1 * SEq_2 + SEq_3 * SEq_4), (SEq_1 * SEq_1) - (SEq_2 * SEq_2) - (SEq_3 * SEq_3) + (SEq_4 * SEq_4));
      }
      
      // Compute heading *BEFORE* the final export of yaw pitch roll to save float computation of deg2rad / rad2deg
      ComputeHeading();
      
      /////////////////////////////////////////////////////////////////////////////////
      // Degree per second conversion and declination correction 
      pitch *= 180.0f / PI;
      yaw   *= 180.0f / PI; 
      yaw   -= DECLINATION; 
      roll  *= 180.0f / PI;

       // Quaternion export
      /*Serial.print("\nQuats = ");
       Serial.println(SEq_1);
       Serial.println(SEq_2);
       Serial.println(SEq_3);
       Serial.println(SEq_4);*/

      char *pData = Quaternions.pData;
      FloatToBigEndian(pData, &SEq_1);
      pData += sizeof(float);
      FloatToBigEndian(pData, &SEq_2);
      pData += sizeof(float);
      FloatToBigEndian(pData, &SEq_3);
      pData += sizeof(float);
      FloatToBigEndian(pData, &SEq_4);
      pData += sizeof(float);

      pData = EulerAngles.pData;
      FloatToBigEndian(pData, &yaw);
      pData += sizeof(float);
      FloatToBigEndian(pData, &pitch);
      pData += sizeof(float);
      FloatToBigEndian(pData, &roll);
      pData += sizeof(float);
      FloatToBigEndian(pData, &heading);

      // Update sensors data in the main OSC message
      pData = RawSensors.pData;

      if(SendCalibrated)
      {
        AccelerationX.Value -= accel_bias[0];
        AccelerationY.Value -= accel_bias[1];
        AccelerationZ.Value -= accel_bias[2];
        GyroscopeX.Value -= gyro_bias[0];
        GyroscopeY.Value -= gyro_bias[1];
        GyroscopeZ.Value -= gyro_bias[2];
        MagnetometerX.Value -= mag_bias[0];
        MagnetometerY.Value -= mag_bias[1];
        MagnetometerZ.Value -= mag_bias[2];
      }

      ShortToBigEndian(pData, BatteryVoltage);
      pData += sizeof(int);
      ShortToBigEndian(pData, SwitchState);
      pData += sizeof(int);      
      WordToBigEndian(pData, AccelerationX);
      pData += sizeof(int);
      WordToBigEndian(pData, AccelerationY);
      pData += sizeof(int);
      WordToBigEndian(pData, AccelerationZ);
      pData += sizeof(int);
      WordToBigEndian(pData, GyroscopeX);
      pData += sizeof(int);
      WordToBigEndian(pData, GyroscopeY);
      pData += sizeof(int);
      WordToBigEndian(pData, GyroscopeZ);
      pData += sizeof(int);
      WordToBigEndian(pData, MagnetometerX);
      pData += sizeof(int);
      WordToBigEndian(pData, MagnetometerY);
      pData += sizeof(int);
      WordToBigEndian(pData, MagnetometerZ);
      // Temperature
      pData += sizeof(int);
      WordToBigEndian(pData, Temperature);
      
         // Optional
      // Use if you want to export analog inputs
      pData = AnalogInputs.pData;
      ShortToBigEndian(pData, AnalogInput1);
      pData += sizeof(int);
      ShortToBigEndian(pData, AnalogInput2);
      pData += sizeof(int);
     
      if(APorStation == AP_MODE)
      {
        // Send packets to *all* connected stations
        unsigned int nbClients;      
        nbClients = WiFi.getTotalDevices();
        if(nbClients > MAX_CLIENTS)
          nbClients = MAX_CLIENTS;     
       
        for(int i = 0 ; i < nbClients ; i++)
        {
          DestIP = WiFi.deviceIpAddress(i);
           // Debug
          //Serial.print("Client #");
          //Serial.print(i);
          //Serial.print(" at IP= ");
          //Serial.println(DestIP);
          //Serial.print(", MAC = ");
          //Serial.println(WiFi.deviceMacAddress(i));
          UdpPacket.beginPacket(DestIP, DestPort);
          
          UdpPacket.write((uint8_t*)AnalogInputs.buf, AnalogInputs.PacketSize);
          UdpPacket.endPacket();

          UdpPacket.write((uint8_t*)RawSensors.buf, RawSensors.PacketSize);
          UdpPacket.endPacket();
          UdpPacket.write((uint8_t*)Quaternions.buf, Quaternions.PacketSize);
          UdpPacket.endPacket();
          // Optional
          UdpPacket.write((uint8_t*)EulerAngles.buf, EulerAngles.PacketSize);
          UdpPacket.endPacket();

          digitalWrite(POWER_LED, LOW);
        }
      }
      else
      {
        UdpPacket.write((uint8_t*)AnalogInputs.buf, AnalogInputs.PacketSize);
        UdpPacket.endPacket();

        UdpPacket.write((uint8_t*)RawSensors.buf, RawSensors.PacketSize);
        UdpPacket.endPacket();
        UdpPacket.write((uint8_t*)Quaternions.buf, Quaternions.PacketSize);
        UdpPacket.endPacket();
        // Optional
        UdpPacket.write((uint8_t*)EulerAngles.buf, EulerAngles.PacketSize);
        UdpPacket.endPacket();
        digitalWrite(POWER_LED, LOW);
       }
    }
  }
  //////////////////////////////////////////////////////////////////////////////////
  // Handles the web server
  else
  {
    if(WiFi.localIP() == INADDR_NONE)
    {        
      if((millis() - ElapsedTime2) > 100) // Blinks faster than during normal mode
      {
        ElapsedTime2 = millis();
        // print dots while we wait to connect and blink the power led
        Serial.print(".");
        if(digitalRead(POWER_LED))
          digitalWrite(POWER_LED, LOW);
        else
          digitalWrite(POWER_LED, HIGH);
      }  
    }

    else if (!statusAP) {
      statusAP = true;
      digitalWrite(POWER_LED, HIGH);
      Serial.println("AP active.");
      printCurrentNet();
      printWifiData();
      Serial.println("Starting webserver on port 80");
      server.begin();
      Serial.println("Webserver started!");
    }

    if(statusAP) // We can accept clients
    {
      char c;
      char LocalHttpBuffer[300];
      int HttpBufferIndex = 0;
      unsigned int Index, Rank;

      client = server.available();

      if (client) {
        Serial.println("new client");
        // an http request ends with a blank line
        boolean currentLineIsBlank = true;
        boolean StayConnected = true;
        
        while (client.connected() && StayConnected )
        {
          if (client.available())
          {

            c = client.read();
            Serial.write(c);

            if(c != '\r')
              LocalHttpBuffer[HttpBufferIndex++] = c;
            if(!currentLineIsBlank && c == '\n')
            { 
              LocalHttpBuffer[HttpBufferIndex++] = '\0';
              // Process HTTP contents
              //Serial.println("new http line - processing");
              //Serial.print(LocalHttpBuffer);
              HttpBufferIndex = 0;
              if(!strncmp(LocalHttpBuffer, "GET / ", 6))
                PageToDisplay = CONFIG_WEB_PAGE;

              else if(!strncmp(LocalHttpBuffer, "GET /params", 11))
              { // Apply settings request - parsing all parameters, stores, update, reboot
                char *pHtml = LocalHttpBuffer;
                // position the pointer on the first param
                while(*pHtml != '?' && *pHtml != '\0')  pHtml++;
                pHtml++;  // skips the ?
                while(*pHtml != '\0')
                {
                  pHtml += GrabNextParam(pHtml, StringBuffer);
                  //Serial.print("Param found: ");
                  //Serial.println(StringBuffer);

                  // Parsing params withing the submitted URL
                  if(!strncmp("ssid", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    strcpy(ssid, &(StringBuffer[Index]));
                    //Serial.print("Updated SSID: ");
                    //Serial.println(ssid);
                  }
                  
                  if(!strncmp("pass", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    
                    strcpy(password, &(StringBuffer[Index]));
                    Serial.print("Updated password: ");
                    Serial.println(password);
                  }
                  
                  if(!strncmp("security", StringBuffer, 8))
                  {
                    Index = SkipToValue(StringBuffer);
                    if(!strncmp(&(StringBuffer[Index]), "WPA2", 4))
                      UseSecurity = true;
                    else
                      UseSecurity = false;
                    
                    Serial.print("Updated Security: ");
                    Serial.println(UseSecurity);
                  }
                  
                  if(!strncmp("mode", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    if(!strncmp(&(StringBuffer[Index]), "station", 7))
                      APorStation = STATION_MODE;
                    else
                      APorStation = AP_MODE;
                    
                    Serial.print("Updated Mode: ");
                    Serial.println(APorStation);
                  }
                  
                  if(!strncmp("type", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    if(!strncmp(&(StringBuffer[Index]), "static", 6))
                      UseDHCP = false;
                    else
                      UseDHCP = true;
                    
                    Serial.print("Updated DHCP: ");
                    Serial.println(UseDHCP);
                  }

                  if(!strncmp("ip", StringBuffer, 2))
                  {
                    Rank = atoi(&(StringBuffer[2]));
                    Index = SkipToValue(StringBuffer);
                    LocalIP[Rank-1] = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("dip", StringBuffer, 3))
                  {
                    Rank = atoi(&(StringBuffer[3]));
                    Index = SkipToValue(StringBuffer);
                    DestIP[Rank-1] = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("gw", StringBuffer, 2))
                  {
                    Rank = atoi(&(StringBuffer[1]));
                    Index = SkipToValue(StringBuffer);
                    GatewayIP[Rank-1] = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("msk", StringBuffer, 3))
                  {
                    Rank = atoi(&(StringBuffer[1]));
                    Index = SkipToValue(StringBuffer);
                    SubnetMask[Rank-1] = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("port", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    DestPort = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("id", StringBuffer, 2))
                  {
                    Index = SkipToValue(StringBuffer);
                    ModuleID = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("rate", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    SampleRate = atoi(&(StringBuffer[Index]));
                  }
                } // End of WHILE(PARSING PARAMETERS)
                // Save Params
                SaveFlashPrefs();
                Serial.println("Params updated and saved");
                PageToDisplay = PARAMS_WEB_PAGE;

              }
            }

            ////////////////////////////////////////////////////////////////////////
            // if you've gotten to the end of the line (received a newline
            // character) and the line is blank, the http request has ended,
            // so you can send a reply
            if (c == '\n' && currentLineIsBlank) 
            {
              Serial.println("sending webpage");
              // send a standard http response header
              switch(PageToDisplay)
              {
              case CONFIG_WEB_PAGE:
                SendConfigWebPage();
                StayConnected = false;
                delay(100);
                break;

              case PARAMS_WEB_PAGE:
                SendParamsWebpage();
                StayConnected = false;
                delay(100);
                break;

              default:
                client.println(HTTP_RESPONSE_0);
                client.println(HTTP_RESPONSE_1);
                client.println(HTTP_RESPONSE_2);
                client.println();
                delay(100);
                StayConnected = false;
                break;
              }
            }
            if (c == '\n')
            {
              // you're starting a new line
              currentLineIsBlank = true;
            }
            else if (c != '\r')
            {
              // you've gotten a character on the current line
              currentLineIsBlank = false;
            }
          }
        }
        // give the web browser time to receive the data
        delay(100);

        // close the connection:
        client.stop();
        Serial.println("client disconnected");
      } 
    }
  }
  
  //////////////////////////////////////////////////////////////////////////////////
  // Incoming serial message (config, control)
  if(GrabSerialMessage())
  {
    //Serial.println("New serial message");
    ProcessSerial();
    FlagSerial = false;
  } 
}


void Connect(void)
{
  if(!UseSecurity)
    WiFi.begin(ssid);
  else
    WiFi.begin(ssid, password); // If security is needed

  // if static IP - this still has problems with profiles and CC3200 API
  if(!UseDHCP)
    WiFi.config(LocalIP, GatewayIP, GatewayIP, SubnetMask);
}


void SendConfigWebPage(void)
{
  client.println(HTTP_RESPONSE_0);
  client.println(HTTP_RESPONSE_1);
  client.println(HTTP_RESPONSE_2);
  client.println();
  for(int i = 0 ; i < HTML_HEADER_CSS_SIZE ; i++)
  {
    client.println(pHTML_HEADER_CSS[i]);
  }
  // IRCAM logo
  for(int i = 0 ; i < IRCAM_LOGO_SIZE ; i++)
  {
    client.print(pIRCAM_LOGO[i]);
    delay(1);           
  }
  client.println();
  client.println("</span></h1>\n<br/><br/><hr>");
  client.println("<h1>R-IoT Configuration Page</h1>");
  client.println("<p><table><tr><td><strong>Module Information</strong></td></tr></table>");
  sprintf(StringBuffer, "<table><tr><td>MAC: %02x:%02x%:%02x:%02x:%02x:%02x</td></tr>\0",mac[0], mac[1], mac[2], mac[3], mac[5], mac[5]);
  client.println(StringBuffer);
  sprintf(StringBuffer,"<tr><td>ID: %u</td></tr>\0",ModuleID);
  client.println(StringBuffer);
  
  client.println("<tr><td>Beta = \0");
  client.println(beta);
  client.println("</td></tr><br/>");
  
  client.println("<tr><td>Zeta = \0");
  client.println(zeta);
  client.println("</td></tr><br/>");
  
  sprintf(StringBuffer, "<tr><td>Firmware: %s</td></tr></table><br/><br/>\0", VERSION_DATE);
  client.println(StringBuffer);
  client.println("<table><tr><td><strong>Network Configuration</strong></td></tr></table>");
  client.println("<form method=\"GET\" action=\"params\"><table><tr><td>");
  
  client.print("<tr><td>WIFI MODE:</td><td><select name=\"mode\">");
  delay(10);
  if(APorStation == STATION_MODE)
    client.println("<option selected=\"selected\">station</option><option>AP</option></select></td></tr>");
  else
    client.println("<option>station</option><option selected=\"selected\">AP</option></select></td></tr>"); 
  delay(10);
  
  client.print("<tr><td>IP TYPE:</td><td><select name=\"type\">");
  delay(10);
  if(!UseDHCP)
    client.println("<option selected=\"selected\">static</option><option>DHCP</option></select></td></tr>");
  else
    client.println("<option>static</option><option selected=\"selected\">DHCP</option></select></td></tr>"); 
  delay(10);
  
  sprintf(StringBuffer,"<tr><td>SSID:</td><td><input type=\"text\" size=\"32\" maxlength=\"32\" name=\"ssid\" value=\"%s\"></td></tr>\0", ssid);
  client.println(StringBuffer);
  
  client.println("<tr><td>SECURITY:</td><td><select name=\"security\">");
  delay(10);
  if(!UseSecurity)
    client.println("<option selected=\"selected\">None</option><option>WPA2</option></select></td></tr>");
  else
    client.println("<option>None</option><option selected=\"selected\">WPA2</option></select></td></tr>");
  
  delay(10);
  client.print("<tr><td>PASSWD:</td><td><input type=\"text\" size=\"32\" maxlength=\"32\" name=\"pass\"");
  delay(10);
  sprintf(StringBuffer, " value=\"%s\"></td></tr>\0", password);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>IP:</td><td><input type=\"text\" size=\"3\" maxlength=\"3\" name=\"ipi1\" value=\"%u\">.\0", LocalIP[0]);
  client.println(StringBuffer);
  sprintf(StringBuffer,"<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"ipi2\" value=\"%u\">.\0", LocalIP[1]);
  client.println(StringBuffer);
  sprintf(StringBuffer,"<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"ipi3\" value=\"%u\">.\0", LocalIP[2]);
  client.println(StringBuffer);
  sprintf(StringBuffer,"<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"ipi4\" value=\"%u\"></td></tr>\0", LocalIP[3]);
  client.println(StringBuffer);

  sprintf(StringBuffer, "<tr><td>DEST IP:</td><td><input type=\"text\" size=\"1\" maxlength=\"3\" name=\"dip1\" value=\"%u\">.\0", DestIP[0]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"dip2\" value=\"%u\">.\0", DestIP[1]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"dip3\" value=\"%u\">.\0", DestIP[2]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"dip4\" value=\"%u\"></td></tr>\0", DestIP[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>GATEWAY:</td><td><input type=\"text\" size=\"1\" maxlength=\"3\" name=\"gw1\" value=\"%u\">.\0", GatewayIP[0]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"gw2\" value=\"%u\">.\0", GatewayIP[1]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"gw3\" value=\"%u\">.\0", GatewayIP[2]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"gw4\" value=\"%u\"></td></tr>\0", GatewayIP[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>MASK:</td><td><input type=\"text\" size=\"1\" maxlength=\"3\" name=\"msk1\" value=\"%u\">.\0", SubnetMask[0]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"msk2\" value=\"%u\">.\0", SubnetMask[1]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"msk3\" value=\"%u\">.\0", SubnetMask[2]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"msk4\" value=\"%u\"></td><br/></tr>\0", SubnetMask[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>PORT:</td><td><input type=\"text\" size=\"4\" maxlength=\"6\" name=\"port\" value=\"%u\"></td></tr>\0", DestPort);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>ID:</td><td><input type=\"text\" size=\"4\" maxlength=\"3\" name=\"id\" value=\"%u\"></td></tr/>\0", ModuleID);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>SAMPLERATE:</td><td><input type=\"text\" size=\"4\" maxlength=\"4\" name=\"rate\" value=\"%u\"></td></tr>\0", SampleRate);
  client.println(StringBuffer);

  client.println("<tr><td><br/></td></tr><tr><td><input type=\"submit\" value=\"Submit\"></td></tr></table></form>");

  // output the value of each analog input pin
  BatteryVoltage = analogRead(BATT_MONITORING);
  client.print("Battery=");
  client.print((float)BatteryVoltage / 801.5);
  client.println(" volts");
  client.println("<br /></body></html>");
  delay(1);
  client.println();
}


void SendParamsWebpage(void)
{
  client.println(HTTP_RESPONSE_0);
  client.println(HTTP_RESPONSE_1);
  client.println(HTTP_RESPONSE_2);
  client.println();
  for(int i = 0 ; i < HTML_HEADER_CSS_SIZE ; i++)
  {
    client.println(pHTML_HEADER_CSS[i]);
  }
  // IRCAM logo
  for(int i = 0 ; i < IRCAM_LOGO_SIZE ; i++)
  {
    client.print(pIRCAM_LOGO[i]);
    delay(1);           
  }
  client.println();
  client.println("</span></h1>\n<br/><br/><hr>");
  client.println("<h1>R-IoT Configuration *SAVED* - OK</h1>");
  client.println("<p><table><tr><td><strong>Module Information</strong></td></tr></table>");
  sprintf(StringBuffer, "<table><tr><td>MAC: %02x:%02x%:%02x:%02x:%02x:%02x</td></tr>\0",mac[0], mac[1], mac[2], mac[3], mac[5], mac[5]);
  client.println(StringBuffer);
  sprintf(StringBuffer,"<tr><td>ID: %u</td></tr><tr><td>\0",ModuleID);
  client.println(StringBuffer);
  sprintf(StringBuffer, "Firmware: %s</td></tr></table><br/><br/>\0", VERSION_DATE);
  client.println(StringBuffer);
  client.println("<table><tr><td><strong>Network Configuration</strong></td></tr></table>");
  client.println("<table><tr><td>");
  
  client.print("<tr><td>WIFI MODE:</td>");
  if(APorStation == STATION_MODE)
    client.println("<td> Station </td></tr>");
  else
    client.println("<td> Access Point </td></tr>");
  client.print("<tr><td>IP TYPE:</td>");
  if(!UseDHCP)
    client.println("<td> Static IP </td></tr>");
  else
    client.println("<td> DHCP </td></tr>");
  sprintf(StringBuffer,"<tr><td>SSID:</td><td>%s</td></tr>\0", ssid);
  client.println(StringBuffer);
  client.println("<tr><td>SECURITY:</td>");
  sprintf(StringBuffer, "<td> %d </td></tr>\0", UseSecurity);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>PASSWD:</td><td> %s </td></tr>\0", password);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>IP:</td><td>%u.%u.%u.%u</td></tr>\0", LocalIP[0], LocalIP[1], LocalIP[2], LocalIP[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>DEST IP:</td><td>%u.%u.%u.%u</td></tr>\0", DestIP[0], DestIP[1], DestIP[2], DestIP[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>GATEWAY:</td><td>%u.%u.%u.%u</td></tr>\0", GatewayIP[0], GatewayIP[1], GatewayIP[2], GatewayIP[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>SUBNET MASK:</td><td>%u.%u.%u.%u</td></tr>\0", SubnetMask[0], SubnetMask[1], SubnetMask[2], SubnetMask[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>PORT:</td><td>%u</td></tr>\0", DestPort);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>MODULE ID:</td><td>%u</td></tr>\0", ModuleID);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>SAMPLE RATE:</td><td>%u</td></tr>\0", SampleRate);
  client.println(StringBuffer);

  client.println("<tr><td><br/></td></tr></table>");

  // output the value of each analog input pin
  BatteryVoltage = analogRead(BATT_MONITORING);
  client.print("Battery=");
  client.print((float)BatteryVoltage / 801.5);
  client.println(" volts");
  client.println("<br /></body></html>"); 
  delay(1);
  client.println();

}

unsigned int GrabNextParam(char *pBuffer, char *ParamString)
{
  unsigned int Len = 0;

  // URL / param string looks like below
  //  /params?type=static+IP&ssid=RIOT-36b9&type=None&pass=&ipi1=192&ipi2=168&ipi3=1&ipi4=1&dip
  // Looks up the & that split between the parameters
  // uses a fair search limit of 50 chars to avoid crashing in case something goes wrong
  while(Len < 50 && *pBuffer != '&' && *pBuffer != ' ')
  {
    ParamString[Len] = *pBuffer;
    pBuffer++;
    Len++;
  }
  pBuffer++; // skips the &
  ParamString[Len] = '\0';  // terminates the string
  Len++;
  return(Len);
}


void printWifiData() {
  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print your MAC address:  
  printf("MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.print("Dest. IP Address: ");
  Serial.println(DestIP);

  // print your subnet mask:
  IPAddress subnet = WiFi.subnetMask();
  Serial.print("NetMask: ");
  Serial.println(subnet);

  // print your gateway address:
  IPAddress gateway = WiFi.gatewayIP();
  Serial.print("Gateway: ");
  Serial.println(gateway);

  printf("UDP/OSC Port=%u\n", DestPort);    
  printf("Module ID=%u\n", ModuleID);    
  printf("Sample Period (ms)=%u\n", SampleRate);
}


void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI) in dB:");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. 

// TODO : implement the fast inverse squareroot to reduce computation time (compare)

void MadgwickQuaternionUpdate(float ax, float ay, float az, float wx, float wy, float wz, float mx, float my, float mz)
{
  // local system variables
  float norm; // vector norm
  float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
  float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
  J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
  float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
  float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
  float h_x, h_y, h_z; // computed flux in the earth frame
  // axulirary variables to avoid reapeated calcualtions
  float halfSEq_1 = 0.5f * SEq_1;
  float halfSEq_2 = 0.5f * SEq_2;
  float halfSEq_3 = 0.5f * SEq_3;
  float halfSEq_4 = 0.5f * SEq_4;
  float twoSEq_1 = 2.0f * SEq_1;
  float twoSEq_2 = 2.0f * SEq_2;
  float twoSEq_3 = 2.0f * SEq_3;
  float twoSEq_4 = 2.0f * SEq_4;
  float twob_x = 2.0f * b_x;
  float twob_z = 2.0f * b_z;
  float twob_xSEq_1 = twob_x * SEq_1;
  float twob_xSEq_2 = twob_x * SEq_2;
  float twob_xSEq_3 = twob_x * SEq_3;
  float twob_xSEq_4 = twob_x * SEq_4;
  float twob_zSEq_1 = twob_z * SEq_1;
  float twob_zSEq_2 = twob_z * SEq_2;
  float twob_zSEq_3 = twob_z * SEq_3;
  float twob_zSEq_4 = twob_z * SEq_4;
  float SEq_1SEq_2;
  float SEq_1SEq_3 = SEq_1 * SEq_3;
  float SEq_1SEq_4;
  float SEq_2SEq_3;
  float SEq_2SEq_4 = SEq_2 * SEq_4;
  float SEq_3SEq_4;
  float twom_x;
  float twom_y;
  float twom_z;

  // normalise the accelerometer measurement
  //norm = AccurateInvSqrt(ax * ax + ay * ay + az * az);
  norm = InvSqrt(ax * ax + ay * ay + az * az);
  ax *= norm;
  ay *= norm;
  az *= norm;

  // normalise the magnetometer measurement
  //norm = AccurateInvSqrt(mx * mx + my * my + mz * mz);
  norm = InvSqrt(mx * mx + my * my + mz * mz);
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Those must be calculated *after* the magnetometers got normalized
  // (mistake in the original paper)
  // Correction suggested 2 * m_x (provided mag) while it's 2 * mx (normalized mag)
  twom_x = 2.0f * mx;
  twom_y = 2.0f * my;
  twom_z = 2.0f * mz;

  // compute the objective function and Jacobian
  f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - ax;
  f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - ay;
  f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - az;
  f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - mx;
  f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - my;
  f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - mz;

  J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
  J_12or23 = 2.0f * SEq_4;
  J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
  J_14or21 = twoSEq_2;
  J_32 = 2.0f * J_14or21; // negated in matrix multiplication
  J_33 = 2.0f * J_11or24; // negated in matrix multiplication
  J_41 = twob_zSEq_3; // negated in matrix multiplication
  J_42 = twob_zSEq_4;
  J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
  J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
  J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
  J_52 = twob_xSEq_3 + twob_zSEq_1;
  J_53 = twob_xSEq_2 + twob_zSEq_4;
  J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
  J_61 = twob_xSEq_3;
  J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
  J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
  J_64 = twob_xSEq_2;

  //compute the gradient (matrix multiplication)
  SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
  SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
  SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
  SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;

  // normalise the gradient to estimate direction of the gyroscope error
  norm = InvSqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
  SEqHatDot_1 = SEqHatDot_1 * norm;
  SEqHatDot_2 = SEqHatDot_2 * norm;
  SEqHatDot_3 = SEqHatDot_3 * norm;
  SEqHatDot_4 = SEqHatDot_4 * norm;

  // compute angular estimated direction of the gyroscope error
  w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
  w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
  w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;

  // compute and remove the gyroscope baises
  w_bx += w_err_x * deltat * zeta;
  w_by += w_err_y * deltat * zeta;
  w_bz += w_err_z * deltat * zeta;
  wx -= w_bx;
  wy -= w_by;
  wz -= w_bz;

  // compute the quaternion rate measured by gyroscopes
  SEqDot_omega_1 = -halfSEq_2 * wx - halfSEq_3 * wy - halfSEq_4 * wz;
  SEqDot_omega_2 = halfSEq_1 * wx + halfSEq_3 * wz - halfSEq_4 * wy;
  SEqDot_omega_3 = halfSEq_1 * wy - halfSEq_2 * wz + halfSEq_4 * wx;
  SEqDot_omega_4 = halfSEq_1 * wz + halfSEq_2 * wy - halfSEq_3 * wx;

  // compute then integrate the estimated quaternion rate
  SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
  SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
  SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
  SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

  // normalise quaternion
  //norm = AccurateInvSqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
  norm = InvSqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
  SEq_1 *= norm;
  SEq_2 *= norm;
  SEq_3 *= norm;
  SEq_4 *= norm;

  // compute flux in the earth frame
  SEq_1SEq_2 = SEq_1 * SEq_2; // recompute axulirary variables
  SEq_1SEq_3 = SEq_1 * SEq_3;
  SEq_1SEq_4 = SEq_1 * SEq_4;
  SEq_3SEq_4 = SEq_3 * SEq_4;
  SEq_2SEq_3 = SEq_2 * SEq_3;
  SEq_2SEq_4 = SEq_2 * SEq_4;

  h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
  h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
  h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);

  // normalise the flux vector to have only components in the x and z
  b_x = sqrt((h_x * h_x) + (h_y * h_y));
  b_z = h_z;

}


//////////////////////////////////////////////////////////////////
// Computation below got adapted from Freescale application note
// for a tilt compensated compass, formerly using accelerometers
// as inclinometers.
// We however directly grab the stable Pitch / Roll angles from Madgwick
// To de-rotate the mag data. This way we are un-sensitive to shaking
// (classic algorithm uses static accel data to get absolute angles)
//
// Beware, the computation below expects angles provided in rad
// so the routine must be called when pitch and roll are still expressed
// in that unit. Cheezy dirty cheap optimization but embedded rulz
void ComputeHeading(void)
{
  float iSin, iCos; /* sine and cosine */
  float iBpx, iBpy, iBpz, LocalPitch, LocalRoll;
  float iBfx, iBfy, iBfz;  // de rotated values of the mag sensors
  	
  // We work with the calibrated values of the MAG sensors
  // (hard iron offset removed + elipsoid fitting matrix)
  iBpx = mag_cal[0];
  iBpy = mag_cal[1];
  iBpz = mag_cal[2];

  LocalRoll = roll;
  LocalPitch = pitch;

  if(SensorOrientation == BOTTOM)
  {
    //LocalRoll = LocalRoll * -1.0;
    LocalPitch = LocalPitch * -1.0;
  }
	
  iBpz = iBpz * -1.0;    // Z mag axis is inverted on the LSM9DS0
  
  /* calculate sin and cosine of roll angle Phi */
  iSin = sin(LocalRoll);
  iCos = cos(LocalRoll); 
  /* de-rotate by roll angle Phi */  
  iBfy = (iBpy * iCos) - (iBpz * iSin);/* Eq 19 y component */
  iBpz = (iBpy * iSin) + (iBpz * iCos);/* Bpy*sin(Phi)+Bpz*cos(Phi)*/

  /* calculate sin and cosine of pitch angle Theta */
  iSin = sin(LocalPitch);
  iCos = cos(LocalPitch);	
  /* de-rotate by pitch angle Theta */
  iBfx = (iBpx * iCos) + (iBpz * iSin); /* Eq 19: x component */
  iBfz = (-iBpx * iSin) + (iBpz * iCos);/* Eq 19: z component */
	
  /* calculate current yaw/heading */
  heading = atan2(-iBfy, iBfx); /* Eq 22 */
  heading = (heading * 180.0) / PI;
}


//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
// (thank you quake)
float InvSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}


// Variant with 1/3 of the error of the code above
// https://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/
float AccurateInvSqrt(float x){
  uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
  float tmp = *(float*)&i;
  return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}


void CalibrateMag(float *CalibratedArray, float *UnCalibratedArray)    
{  
  //calculation (bias was removed right after data acquisition)
  float result[3] = {
    0.,  0., 0.    };  
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += rotation_matrix[i][j] * UnCalibratedArray[j];
  for (int i=0; i<3; ++i) 
    CalibratedArray[i] = result[i];
}

boolean GrabSerialMessage(void)
{
  char TheChar;
  if (Serial.available())
  {
    while(Serial.available() && (SerialIndex < MAX_SERIAL))
    {
      TheChar = Serial.read();
      //Serial.print(TheChar);
      if((TheChar == '\n') || (TheChar == '\r'))
      {
        SerialBuffer[SerialIndex] = '\0';
        SerialIndex = 0;
        strcpy(StringBuffer, SerialBuffer);
        FlagSerial = true; 
        return(true);
      }
      else
      {
        SerialBuffer[SerialIndex++] = TheChar;
      }
    }
    if(SerialIndex >= MAX_SERIAL)
      SerialIndex = 0;
  }
  return(false);
}

void ProcessSerial(void)
{
  unsigned char i;
  unsigned char Index;

  byte temp_ip[4];

  // Debug
  //printf("Cmd: %s - OK\n",StringBuffer);

  // Send current config to the configuration app
  if(!strncmp("cfgrequest",StringBuffer,10))
  {
    // Outputs all the configuration  
    printf("%s %d\n", TEXT_DHCP, UseDHCP);
    printf("%s %s\n", TEXT_SSID, ssid);
    printf("%s %s\n", TEXT_WIFI_MODE, APorStation);
    printf("%s %d\n", TEXT_SECURITY, UseSecurity);
    printf("%s %s\n", TEXT_PASSWORD, password);
    printf("%s %u.%u.%u.%u\n", TEXT_OWNIP, LocalIP[0], LocalIP[1], LocalIP[2], LocalIP[3] );
    printf("%s %u.%u.%u.%u\n", TEXT_DESTIP, DestIP[0], DestIP[1], DestIP[2], DestIP[3]);
    printf("%s %u.%u.%u.%u\n", TEXT_GATEWAY, GatewayIP[0], GatewayIP[1],GatewayIP[2],GatewayIP[3]);
    printf("%s %u.%u.%u.%u\n", TEXT_MASK, SubnetMask[0], SubnetMask[1], SubnetMask[2], SubnetMask[3] );
    printf("%s %u\n", TEXT_PORT, DestPort);
    printf("%s %u\n", TEXT_MASTER_ID, ModuleID);
    printf("%s %u\n", TEXT_SAMPLE_RATE, SampleRate);
    
    // All offsets as lists + rotation matrix
    
    
    printf("refresh\n");
  }

  // Ping / Echo question/answer from the GUI
  else if(!strncmp("ping",StringBuffer,4))
  {
    printf("echo\n");	// a simple ASCII echo answer to let the GUI know the COM port is the right one
    return;
  }

  else if(!strncmp("savecfg",StringBuffer,7))	// Saves config to FLASH
  {
    SaveFlashPrefs();
    // Reboot is needed to use new settings
    //Reboot();	
  }
  
  else if(!strncmp(TEXT_WIFI_MODE,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    sscanf(&StringBuffer[Index],"%d", &APorStation);
    return;	
  }
  
  else if(!strncmp(TEXT_DHCP,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    sscanf(&StringBuffer[Index],"%d", &UseDHCP);
    return;	
  }
  else if(!strncmp(TEXT_SSID,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    memset(ssid, '\0', sizeof(ssid));
    sscanf(&StringBuffer[Index],"%s", ssid);
    return;	
  }
  else if(!strncmp(TEXT_OWNIP,StringBuffer,5))
  {
    Index = SkipToValue(StringBuffer);
    //printf("%s\n",&StringBuffer[Index]);
    ParseIP(&StringBuffer[Index], &LocalIP);
    //printf("own ip update %u.%u.%u.%u\n",pucIP_Addr[0], pucIP_Addr[1], pucIP_Addr[2], pucIP_Addr[3]);
    return;
  }	
  else if(!strncmp(TEXT_DESTIP,StringBuffer,6))
  {
    Index = SkipToValue(StringBuffer);
    ParseIP(&StringBuffer[Index], &DestIP);
    return;
  }	
  else if(!strncmp(TEXT_GATEWAY,StringBuffer,7))
  {
    Index = SkipToValue(StringBuffer);
    ParseIP(&StringBuffer[Index], &GatewayIP);
    return;
  }	
  else if(!strncmp(TEXT_DNS,StringBuffer,3))
  {
    Index = SkipToValue(StringBuffer);
    ParseIP(&StringBuffer[Index], &GatewayIP);
    return;
  }	
  else if(!strncmp(TEXT_MASK,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    ParseIP(&StringBuffer[Index], &SubnetMask);
    return;
  }	
  else if(!strncmp(TEXT_PORT,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    DestPort = atoi(&StringBuffer[Index]);
    return;
  }	
  else if(!strncmp(TEXT_MASTER_ID,StringBuffer,8))
  {
    Index = SkipToValue(StringBuffer);
    ModuleID = atoi(&StringBuffer[Index]);
    return;
  }
  else if(!strncmp(TEXT_SAMPLE_RATE,StringBuffer,10))
  {
    Index = SkipToValue(StringBuffer);
    SampleRate = atoi(&StringBuffer[Index]);
    if(SampleRate < MIN_SAMPLE_RATE)
      SampleRate = MIN_SAMPLE_RATE;

    if(SampleRate > MAX_SAMPLE_RATE)
      SampleRate = MAX_SAMPLE_RATE;

    return;
  }	

  else if(!strncmp("defaults",StringBuffer,8))
  {
    // Re open in write mode
    SerFlash.close();
    if(SerFlash.open(PARAMS_FILENAME, FS_MODE_OPEN_WRITE))
    {
      Serial.println("Restoring defaults");
      RestoreDefaults();
      SerFlash.close();
      Serial.println("Please Reboot");
      while(1);
    }
    //Reboot();	
  }
  else if(!strncmp("reboot",StringBuffer,6))
  {
    // Still needs work to find out how to reboot the chip
    // see hibernate modes)
    //Reboot();
  }

  else if(!strncmp("calibrate",StringBuffer,9))
  {
    Calibrate();
  }

  else if(!strncmp("offset_gyrZ",StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    gyro_bias[2] = atoi(&StringBuffer[Index]);
    gbias[2] = (float)gyro_bias[2]*gRes;
  }		
}

void LoadParams(void)
{
  // Check if file exists
  if(!SerFlash.open(PARAMS_FILENAME, FS_MODE_OPEN_READ))
  {
    SerFlash.close();
    // Creates the file
    Serial.println("Param File not found");
    if(SerFlash.open(PARAMS_FILENAME, FS_MODE_OPEN_CREATE(512, _FS_FILE_OPEN_FLAG_COMMIT)))
    {
      // Re open in write mode
      Serial.println("Param File created and opened for writing");
      Serial.println("Restoring defaults");
      RestoreDefaults();
      SerFlash.close();
      Serial.println("Please Reboot");
      while(1);
      // REBOOT NEEDED 
    }
  }
  else
  {
    Serial.println("Found Param file, parsing");
    int FormatToken;
    GrabLine(StringBuffer);
    // checks if the file is properly formatted with the 0x55 header token
    if(strncmp(StringBuffer, "0x55", 4))
    {
      Serial.println("Restoring defaults");
      SerFlash.close();
      SerFlash.open(PARAMS_FILENAME, FS_MODE_OPEN_WRITE);
      RestoreDefaults();
      SerFlash.close();
      Serial.println("Reboot Device");
    }
    else
    {
      // Parses the preferences
      GrabLine(StringBuffer);
      strcpy(ssid, StringBuffer);
      APorStation = atoi(StringBuffer);
      printf("Wifi Mode = ");
      if(APorStation == STATION_MODE)
        printf("Station\n");
      else
        printf("Access Point\n");
      
      GrabLine(StringBuffer);
      strcpy(ssid, StringBuffer);
      GrabLine(StringBuffer);
      UseSecurity = atoi(StringBuffer);
      printf("WiFi Encryption = %d\n", UseSecurity);
      GrabLine(StringBuffer);
      strcpy(password, StringBuffer);
      printf("WiFi Password = %s\n", password);
      GrabLine(StringBuffer);
      UseDHCP = atoi(StringBuffer);
      printf("Use DHCP = %d\n", UseDHCP);
      GrabLine(StringBuffer);
      ParseIP(StringBuffer, &LocalIP);
      GrabLine(StringBuffer);
      ParseIP(StringBuffer, &DestIP);
      GrabLine(StringBuffer);
      ParseIP(StringBuffer, &GatewayIP);
      GrabLine(StringBuffer);
      ParseIP(StringBuffer, &SubnetMask);
      GrabLine(StringBuffer);
      DestPort = atoi(StringBuffer);      
      GrabLine(StringBuffer);
      ModuleID = atoi(StringBuffer);      
      GrabLine(StringBuffer);
      SampleRate = atoi(StringBuffer);

      if(!SampleRate)
      {
        Serial.println("Min Sample Rate is 3 ms");
        SampleRate = MIN_SAMPLE_RATE;
      }
      deltat = (float)SampleRate / 1000.0f;

      // Loading calibration data
      GrabLine(StringBuffer);
      accel_bias[0] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      accel_bias[1] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      accel_bias[2] = atoi(StringBuffer);
      printf("Loaded Accel Offsets: %d %d %d\n", accel_bias[0], accel_bias[1], accel_bias[2]);

      GrabLine(StringBuffer);
      gyro_bias[0] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      gyro_bias[1] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      gyro_bias[2] = atoi(StringBuffer);
      printf("Loaded Gyro Offsets: %d %d %d\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

      GrabLine(StringBuffer);
      mag_bias[0] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      mag_bias[1] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      mag_bias[2] = atoi(StringBuffer);
      printf("Loaded Mag Offsets: %d %d %d\n", mag_bias[0], mag_bias[1], mag_bias[2]);

      // Calibration matrix
      for(int i = 0 ; i < 3 ; i++)
      {
        for(int j = 0 ; j < 3 ; j++)
        {
          GrabLine(StringBuffer);
          //printf("debug string = %s\n", StringBuffer);
          rotation_matrix[i][j] = atof(StringBuffer);
        }
      }
      printf("Loaded Calibration Matrix \n");
      printf("float rotation_matrix[3][3] = \n");
      printf("[\n");
      for(int i=0 ; i<3 ; i++)
      { 
        Serial.print("\t[");
        for(int j=0 ; j <3 ; j++)
        {
          Serial.print(rotation_matrix[i][j]);
          Serial.print("   ");
        }
        Serial.println("]");
      }
      printf("]\n\n");
      printf("Madgwick Specifics\n");
      Serial.print("Beta=");
      Serial.println(beta);
      Serial.print("Zeta = ");
      Serial.println(zeta);
      SerFlash.close();
    }
  }
}

void SaveFlashPrefs(void)
{
  int writeStatus;

  // File was opened for Read so far, re open in write mode
  SerFlash.close();
  if(SerFlash.open(PARAMS_FILENAME, FS_MODE_OPEN_WRITE))
  {
    Serial.println("Saving prefs in FLASH");

    // Format token 0x55
    writeStatus = SerFlash.write((uint8_t*)("0x55\n"),5);
    
    // Station or AP mode
    sprintf(StringBuffer, "%d\n", APorStation);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    
    // ssid
    sprintf(StringBuffer, "%s\n", ssid);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    // use security ?
    sprintf(StringBuffer, "%d\n", UseSecurity);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    
    sprintf(StringBuffer, "%s\n", password);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%d\n", UseDHCP);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u.%u.%u.%u\n", LocalIP[0], LocalIP[1], LocalIP[2], LocalIP[3]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u.%u.%u.%u\n", DestIP[0], DestIP[1], DestIP[2], DestIP[3]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u.%u.%u.%u\n", GatewayIP[0], GatewayIP[1],GatewayIP[2],GatewayIP[3]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u.%u.%u.%u\n", SubnetMask[0], SubnetMask[1],SubnetMask[2],SubnetMask[3]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u\n", DestPort);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u\n", ModuleID);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u\n", SampleRate);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    // Calibration data
    // Accel offset
    sprintf(StringBuffer, "%d\n", accel_bias[0]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", accel_bias[1]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", accel_bias[2]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    // Gyro offset
    sprintf(StringBuffer, "%d\n", gyro_bias[0]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", gyro_bias[1]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", gyro_bias[2]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    // Mag offset
    sprintf(StringBuffer, "%d\n", mag_bias[0]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", mag_bias[1]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", mag_bias[2]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer)); 

    // Calibration matrix
    for(int i = 0 ; i < 3 ; i++)
    {
      for(int j = 0 ; j < 3 ; j++)
      {
        ftoa(rotation_matrix[i][j], StringBuffer, 5); // No float support for sprintf, using a simple ftoa
        sprintf(StringBuffer, "%s\n", StringBuffer);
        //printf("debug string = %s\n", StringBuffer);
        writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));      
      }
    }

    SerFlash.close();
    Serial.println("Done");
  }
  else
    Serial.println("Error saving params");
}


// Assumes the file is already opened for writing
void RestoreDefaults(void)
{
  int writeStatus;
  
  // Clears the "file"
  //for(int i = 0 ; i < 500 ; i++)
  //  writeStatus = SerFlash.write((uint8_t*)0x00,1);
  
  // Format token 0x55
  writeStatus = SerFlash.write((uint8_t*)("0x55\n"),5);
  
  // Use Station mode
  sprintf(StringBuffer, "%d\n", STATION_MODE);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
  
  // ssid
  sprintf(StringBuffer, "%s\n", TheSSID);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Use Security = false
  sprintf(StringBuffer, "%d\n", false);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
  
  // default password = 12345678
  sprintf(StringBuffer, "12345678\n");
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Use DHCP = true
  sprintf(StringBuffer, "%d\n", true);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Fixed / static IP
  sprintf(StringBuffer, "%u.%u.%u.%u\n", TheLocalIP[0], TheLocalIP[1], TheLocalIP[2], TheLocalIP[3]);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Destination computer IP
  sprintf(StringBuffer, "%u.%u.%u.%u\n", TheDestIP[0], TheDestIP[1], TheDestIP[2], TheDestIP[3]);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  sprintf(StringBuffer, "%u.%u.%u.%u\n", TheGatewayIP[0], TheGatewayIP[1],TheGatewayIP[2],TheGatewayIP[3]);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
  
  sprintf(StringBuffer, "%u.%u.%u.%u\n", TheSubnetMask[0], TheSubnetMask[1],TheSubnetMask[2],TheSubnetMask[3]);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  sprintf(StringBuffer, "%u\n", TheDestPort);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
  
  sprintf(StringBuffer, "%u\n", TheID);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  sprintf(StringBuffer, "%u\n", TheSampleRate);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Calibration data
  // Accel offset
  sprintf(StringBuffer, "0\n");
  // reseting all offsets to zero
  for(int i=0 ; i < 9 ; i++)
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Calibration matrix (set to identity as default)
  for(int i = 0 ; i < 3 ; i++)
  {
    for(int j = 0 ; j < 3 ; j++)
    {
      ftoa(identity_matrix[i][j], StringBuffer, 5);
      sprintf(StringBuffer, "%s\n", StringBuffer);
      //printf("debug string = %s\n", StringBuffer);
      writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));      
    }
  }
}

void EmptyString(char* TheString, int size)
{
  for(int i = 0 ; i < size ; i++)
    TheString[i] = '\0';
}



void Calibrate(void)
{

  unsigned char NextFlag = FALSE;
  unsigned char i;

  // TODO : do a long calibration and an estimate of the gyro drift based 
  // on temperature.
  // Add a real calibration (elipsoid fitting) of the mag sensors

  digitalWrite(POWER_LED, LOW);
  Serial.print("Entering calibration mode\n");
  Serial.print("...\nAccelerometer / Gyro Calibration\n");
  Serial.print("Place Module on a flat surface\n");
  Serial.print("then press the switch or send the \"next\" command via serial\n");

  // Wait for the switch to be depressed or the next command to be sent
  // via serial  
  while(digitalRead(SWITCH_INPUT) && !NextFlag)
  {
    NextFlag = WaitSerialNext(); 
    if(digitalRead(POWER_LED))
      digitalWrite(POWER_LED, LOW);
    else
      digitalWrite(POWER_LED, HIGH);
    delay(100);
  }

  NextFlag = false;
  // Wait for the switch to be released
  digitalWrite(POWER_LED, HIGH);
  while(!digitalRead(SWITCH_INPUT))
  {
    delay(20);
  }

  Serial.print("*** STARTING CALIBRATION ACC/GYR ***\"\n");	

  for(i=0 ; i < 3 ; i++)
  {
    gyro_bias[i] = 0;
    accel_bias[i] = 0;
  }

  // Average the accel and gyro offsets
  for(i=0;i<bias_samples;i++)    // We take some readings...
  {
    if(digitalRead(POWER_LED))
      digitalWrite(POWER_LED, LOW);
    else
      digitalWrite(POWER_LED, HIGH);
    ReadGyro();
    ReadAccel();
    gyro_bias[0] += GyroscopeX.Value;
    gyro_bias[1] += GyroscopeY.Value;
    gyro_bias[2] += GyroscopeZ.Value;
    accel_bias[0] += AccelerationX.Value;
    accel_bias[1] += AccelerationY.Value;
    accel_bias[2] += AccelerationZ.Value - (int)(1./aRes);
    delay(20);
  }

  accel_bias[0] /= bias_samples; // average the data
  accel_bias[1] /= bias_samples; 
  accel_bias[2] /= bias_samples; 

  gyro_bias[0] /= bias_samples; // average the data
  gyro_bias[1] /= bias_samples; 
  gyro_bias[2] /= bias_samples; 

  printf("*** FOUND Bias acc= %d %d %d\n", accel_bias[0], accel_bias[1], accel_bias[2]);
  printf("\*** FOUND Bias gyro= %d %d %d\n\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

  Serial.print("Acceleration / Gyroscope calibration finished\n\n");
  Serial.println("\n\nEntering Calibration of the Magnetometers");
  Serial.println("Press the switch to start or send the \"next\" command via serial");
  while(digitalRead(SWITCH_INPUT) && !NextFlag)
  {
    NextFlag = WaitSerialNext();
    delay(200);
    Serial.print(".");
  }
  Serial.println(" ");

  // Wait for the switch to be released
  digitalWrite(POWER_LED, HIGH);
  while(!digitalRead(SWITCH_INPUT))
  {
    delay(20);
  }

  boolean QuitLoop = false;
  while(!QuitLoop)
  {
    // 100 ms sample rate, we don't need much there
    if((millis() - ElapsedTime) >= 200)
    {       
      ElapsedTime = millis();
      digitalWrite(POWER_LED, HIGH);
      ReadMagneto();     

      if(!digitalRead(SWITCH_INPUT) || NextFlag)
      {
        while(!digitalRead(SWITCH_INPUT))
          delay(20);

        NextFlag = false;

        switch(CalibrationStage)
        {
        case START :
          PrintSensorOrientation("X+ 0deg");
          break;

          // X axis 
        case X_PLUS_0:
          StoreMag(Xplus0);
          printf("X+ 0deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          PrintSensorOrientation("X+ 180deg");
          break;

        case X_PLUS_180:
          StoreMag(Xplus180);
          printf("X+ 180deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          PrintSensorOrientation("X- 0deg");
          break;

        case X_MINUS_0:
          StoreMag(Xminus0);
          printf("X- 0deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          PrintSensorOrientation("X- 180deg");
          break;

        case X_MINUS_180:
          StoreMag(Xminus180);
          printf("X- 180deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          PrintSensorOrientation("Y+ 0deg");
          break;

          // Y axis  
        case Y_PLUS_0:
          StoreMag(Yplus0);
          printf("Y+ 0deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          PrintSensorOrientation("Y+ 180deg");
          break;

        case Y_PLUS_180:
          StoreMag(Yplus180);
          printf("Y+ 180deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          PrintSensorOrientation("Y- 0deg");
          break;

        case Y_MINUS_0:
          StoreMag(Yminus0);
          printf("Y- 0deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          PrintSensorOrientation("Y- 180deg");
          break;

        case Y_MINUS_180:
          StoreMag(Yminus180);
          printf("Y- 180deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          PrintSensorOrientation("Z+ 0deg");
          break;  

          // Z axis  
        case Z_PLUS_0:
          StoreMag(Zplus0);
          printf("Z+ 0deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          PrintSensorOrientation("Z+ 180deg");
          break;

        case Z_PLUS_180:
          StoreMag(Zplus180);
          printf("Z+ 180deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          PrintSensorOrientation("Z- 0deg");
          break;

        case Z_MINUS_0:
          StoreMag(Zminus0);
          printf("Z- 0deg : %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          PrintSensorOrientation("Z- 180deg");
          break;

        case Z_MINUS_180:
          StoreMag(Zminus180);
          printf("Z- 180deg: %d %d %d\n", MagnetometerX.Value, MagnetometerY.Value, MagnetometerZ.Value);
          Serial.println("Calibration Sequence finished");
          break; 

        default:
          break; 
        }

        CalibrationStage++;
        if(CalibrationStage >= END)
        {
          Serial.println("Computing Matrix and Bias");
          CalculateTransformationMatrix();

          //Display results
          //Transformation matrix
          printf("Matrix \n");
          printf("float rotation_matrix[3][3] = \n");
          printf("[\n");
          for(int i=0 ; i<3 ; i++)
          { 
            Serial.print("\t[");
            for(int j=0 ; j <3 ; j++)
            {
              Serial.print(rotation_matrix[i][j]);
              Serial.print("   ");
            }
            Serial.println("]");
          }
          printf("]\n\n");
          // Bias
          printf("Bias\n");
          printf("mag_bias[3] = {%d, %d, %d}\n", mag_bias[0], mag_bias[1], mag_bias[2]);            
          digitalWrite(POWER_LED, HIGH);
          SaveFlashPrefs();	
          Serial.println("\n\n******* CALIBRATION FINISHED - PREFS SAVED - PLEASE REBOOT *********");
          QuitLoop = true;
        }
      }
      digitalWrite(POWER_LED, LOW);
      NextFlag = WaitSerialNext();  
    }
  } // End of while(!QUIT CALIBRATION LOOP)          
  while(1)
  {
    delay(250);
    if(digitalRead(POWER_LED))
      digitalWrite(POWER_LED, LOW);
    else
      digitalWrite(POWER_LED, HIGH);     
  } 
}

/////////////////////////////////////////////////////////////////////////////
// Linear algebra routines to solve the mag sensor matrix inversion

void PrintSensorOrientation(const char *msg)
{
  Serial.print("Place the sensor in the ");
  Serial.print(msg);
  Serial.println(" orientation");
  Serial.println("and press the switch or send the \"next\" command via serial");
}

void StoreMag(float *StorageVector)
{ 
  StorageVector[0] = (float)MagnetometerX.Value;
  StorageVector[1] = (float)MagnetometerY.Value;  
  StorageVector[2] = (float)MagnetometerZ.Value;
}

void CalculateTransformationMatrix(void)
{
  //Axis X--------------------------------------------------------------------------------------------------
  float Xplus_center[3];
  //Centers of the circles 
  Xplus_center[X] = (Xplus0[X] + Xplus180[X]) / 2;
  Xplus_center[Y] = (Xplus0[Y] + Xplus180[Y]) / 2;
  Xplus_center[Z] = (Xplus0[Z] + Xplus180[Z]) / 2;

  //Centers of the circles
  float Xminus_center[3];
  Xminus_center[X] = (Xminus0[X] + Xminus180[X]) / 2;
  Xminus_center[Y] = (Xminus0[Y] + Xminus180[Y]) / 2;
  Xminus_center[Z] = (Xminus0[Z] + Xminus180[Z]) / 2;

  //Vector from the center of minus circle to the center of plus circle
  float Xvector[3];
  Xvector[X] = Xplus_center[X] - Xminus_center[X];
  Xvector[Y] = Xplus_center[Y] - Xminus_center[Y];
  Xvector[Z] = Xplus_center[Z] - Xminus_center[Z];  

  //Axis Y--------------------------------------------------------------------------------------------------
  float Yplus_center[3];
  //Centers of the circles
  Yplus_center[X] = (Yplus0[X] + Yplus180[X]) / 2;
  Yplus_center[Y] = (Yplus0[Y] + Yplus180[Y]) / 2;
  Yplus_center[Z] = (Yplus0[Z] + Yplus180[Z]) / 2;
  //Centers of the circles
  float Yminus_center[3];
  Yminus_center[X] = (Yminus0[X] + Yminus180[X]) / 2;
  Yminus_center[Y] = (Yminus0[Y] + Yminus180[Y]) / 2;
  Yminus_center[Z] = (Yminus0[Z] + Yminus180[Z]) / 2;
  //Vector from the center of minus circle to the center of plus circle
  float Yvector[3];
  Yvector[X] = Yplus_center[X] - Yminus_center[X];
  Yvector[Y] = Yplus_center[Y] - Yminus_center[Y];
  Yvector[Z] = Yplus_center[Z] - Yminus_center[Z];

  //Axis Z--------------------------------------------------------------------------------------------------
  float Zplus_center[3];
  //Centers of the circles
  Zplus_center[X] = (Zplus0[X] + Zplus180[X]) / 2;
  Zplus_center[Y] = (Zplus0[Y] + Zplus180[Y]) / 2;
  Zplus_center[Z] = (Zplus0[Z] + Zplus180[Z]) / 2;
  //Centers of the circles
  float Zminus_center[3];
  Zminus_center[X] = (Zminus0[X] + Zminus180[X]) / 2;
  Zminus_center[Y] = (Zminus0[Y] + Zminus180[Y]) / 2;
  Zminus_center[Z] = (Zminus0[Z] + Zminus180[Z]) / 2;
  //Vector from the center of minus circle to the center of plus circle
  float Zvector[3];
  Zvector[X] = Zplus_center[X] - Zminus_center[X];
  Zvector[Y] = Zplus_center[Y] - Zminus_center[Y];
  Zvector[Z] = Zplus_center[Z] - Zminus_center[Z];

  //Dividing by main value, for example for X axis - dividing by X coordinate, for Y axis by Y coordinate, for Z axis by Z coordinate
  rotation_matrix[0][0] = Xvector[0] / Xvector[0]; 
  rotation_matrix[0][1] = Yvector[0] / Yvector[1]; 
  rotation_matrix[0][2] = Zvector[0] / Zvector[2];
  rotation_matrix[1][0] = Xvector[1] / Xvector[0]; 
  rotation_matrix[1][1] = Yvector[1] / Yvector[1]; 
  rotation_matrix[1][2] = Zvector[1] / Zvector[2];
  rotation_matrix[2][0] = Xvector[2] / Xvector[0]; 
  rotation_matrix[2][1] = Yvector[2] / Yvector[1]; 
  rotation_matrix[2][2] = Zvector[2] / Zvector[2];

  //Matrix inversion
  Serial.println("Inverting Matrix");
  InvertMatrix(rotation_matrix, rotation_matrix_result);
  Serial.println("Invertion done");

  //Determinating the corrected by rotation matrix centers of the circles 
  float CenterResultXplus[3], CenterResultXminus[3];
  float CenterResultYplus[3], CenterResultYminus[3];
  float CenterResultZplus[3], CenterResultZminus[3];

  MatrixVectorMultiply(rotation_matrix, Xplus_center, CenterResultXplus);
  MatrixVectorMultiply(rotation_matrix, Xminus_center, CenterResultXminus);
  MatrixVectorMultiply(rotation_matrix, Yplus_center, CenterResultYplus);
  MatrixVectorMultiply(rotation_matrix, Yminus_center, CenterResultYminus);
  MatrixVectorMultiply(rotation_matrix, Zplus_center, CenterResultZplus);
  MatrixVectorMultiply(rotation_matrix, Zminus_center, CenterResultZminus);

  //Determinating of the elipsoid center---------------------------------------------------------------------------
  float center[3];
  center[X] = (Xplus_center[0] + Xminus_center[0] + Yplus_center[0] + Yminus_center[0] + Zplus_center[0] + Zminus_center[0]) / 6;
  center[Y] = (Xplus_center[1] + Xminus_center[1] + Yplus_center[1] + Yminus_center[1] + Zplus_center[1] + Zminus_center[1]) / 6;
  center[Z] = (Xplus_center[2] + Xminus_center[2] + Yplus_center[2] + Yminus_center[2] + Zplus_center[2] + Zminus_center[2]) / 6;

  //Determinating of the radius of the future sphere-----------------------------------------------------------------------
  float x_length = fabs(Xplus_center[0] - Xminus_center[0])/2;
  float y_length = fabs(Yplus_center[1] - Yminus_center[1])/2;
  float z_length = fabs(Zplus_center[2] - Zminus_center[2])/2;
  float Xplus_0[3];
  Xplus_0[0] =  Xplus0[X]; 
  Xplus_0[1] =  Xplus0[Y]; 
  Xplus_0[2] =  Xplus0[Z];
  MatrixVectorMultiply(rotation_matrix, Xplus0, Xplus_0);
  float Yplus_0[3];
  Yplus_0[0] =  Yplus0[X];
  Yplus_0[1] =  Yplus0[Y];
  Yplus_0[2] =  Yplus0[Z];
  MatrixVectorMultiply(rotation_matrix, Yplus0, Yplus_0);
  float Zplus_0[3];
  Zplus_0[0] =  Zplus0[X];
  Zplus_0[1] =  Zplus0[Y];
  Zplus_0[2] =  Zplus0[Z];
  MatrixVectorMultiply(rotation_matrix, Zplus0, Zplus_0);
  float x_abs = sqrt(x_length * x_length + Xplus0[Y] * Xplus0[Y] + Xplus0[Z] * Xplus0[Z]);
  float y_abs = sqrt(Yplus0[X] * Yplus0[X] + y_length * y_length + Yplus0[Z] * Yplus0[Z]);
  float z_abs = sqrt(Zplus0[X] * Zplus0[X] + Zplus0[Y] * Zplus0[Y] + z_length * z_length);
  //sphere radius
  float sphere_radius = (x_abs + y_abs + z_abs) / 3;

  //Scales for the each axis------------------------------------------------
  //Diameter of the sphere
  float diameter = sphere_radius * 2;
  float kx = fabs(diameter / (Xplus_center[X] - Xminus_center[X]));
  float ky = fabs(diameter / (Yplus_center[Y] - Yminus_center[Y]));
  float kz = fabs(diameter / (Zplus_center[Z] - Zminus_center[Z]));

  //Multiplying elements of matrix by scales
  rotation_matrix[0][0] = rotation_matrix[0][0] * kx;
  rotation_matrix[0][1] = rotation_matrix[0][1] * ky;
  rotation_matrix[0][2] = rotation_matrix[0][2] * kz;
  rotation_matrix[1][0] = rotation_matrix[1][0] * kx; 
  rotation_matrix[1][1] = rotation_matrix[1][1] * ky; 
  rotation_matrix[1][2] = rotation_matrix[1][2] * kz;
  rotation_matrix[2][0] = rotation_matrix[2][0] * kx; 
  rotation_matrix[2][1] = rotation_matrix[2][1] * ky; 
  rotation_matrix[2][2] = rotation_matrix[2][2] * kz;

  //Bias
  mag_bias[X] = (int)center[X];
  mag_bias[Y] = (int)center[Y];
  mag_bias[Z] = (int)center[Z];  

}

///////////////////////////////////////////////////////////////////////////
// Cheap and dirty algebra function designed for 3x3

void MatrixVectorMultiply(float matrixA[][3], float *vectorB, float *result)
{
  int aRows = 3; 
  int aCols = 3;
  int bRows = 3;

  for (int i = 0; i < aRows; ++i) // each row of A
    for (int k = 0; k < aCols; ++k)
      result[i] += matrixA[i][k] * vectorB[k];
}

// A = matric to invert, x = resulting matrix
void InvertMatrix(float A[][3], float x[][3])
{
  int n = 3;
  //e will represent each column in the identity matrix
  float e[3];

  //x will hold the inverse matrix to be returned

  /*
  * solve will contain the vector solution for the LUP decomposition as we solve
   * for each vector of x.  We will combine the solutions into the float[][] array x.
   * */
  float solve[3];

  //Get the LU matrix and P matrix (as an array)
  float LU[3][3];
  int P[3];

  //Serial.println("Before LUP Decomposition");
  LUPDecomposition(A, LU, P);

  /*
  * Solve AX = e for each column e[i] of the identity matrix using LUP decomposition
   * */
  for (int i = 0; i < n; i++)
  {
    // Make an identity matrix column based on the index i
    // clears first
    for(int t = 0 ; t < 3 ; t++)
      e[t] = 0.0;
    e[i] = 1.0;

    LUPSolve(LU, P, e, solve);
    for (int j = 0; j < 3; j++)
    {
      x[j][i] = solve[j];
    }
  }
}


void LUPSolve(float LU[][3], int *pi, float *b, float *solution)
{
  int n = 2; // size-1
  float x[3];
  float y[3];
  float suml = 0;
  float sumu = 0;
  float lij = 0;

  /*
  * Solve for y using formward substitution
   * */
  for (int i = 0; i <= n; i++)
  {
    suml = 0;
    for (int j = 0; j <= i - 1; j++)
    {
      /*
      * Since we've taken L and U as a singular matrix as an input
       * the value for L at index i and j will be 1 when i equals j, not LU[i][j], since
       * the diagonal values are all 1 for L.
       * */
      if (i == j)
      {
        lij = 1;
      }
      else
      {
        lij = LU[i][j];
      }
      suml = suml + (lij * y[j]);
    }
    y[i] = b[pi[i]] - suml;
  }
  //Solve for x by using back substitution
  for (int i = n; i >= 0; i--)
  {
    sumu = 0;
    for (int j = i + 1; j <= n; j++)
    {
      sumu = sumu + (LU[i][j] * x[j]);
    }
    x[i] = (y[i] - sumu) / LU[i][i];
  }

}

void LUPDecomposition(float A[][3], float LU[][3], int  *pi)
{
  int n = 2;
  /*
   * pi represents the permutation matrix.  We implement it as an array
   * whose value indicates which column the 1 would appear.  We use it to avoid 
   * dividing by zero or small numbers.
   * */
  float p = 0;
  int kp = 0;
  int pik = 0;
  int pikp = 0;
  float aki = 0;
  float akpi = 0;

  //Initialize the permutation matrix, will be the identity matrix
  for (int j = 0; j <= n; j++)
  {
    pi[j] = j;
  }

  // We make a copy of the matrix A into LU and work directly on LU for permutations
  // and decomposition.
  for(int i = 0 ; i <= n ; i++)
  {
    for(int j = 0 ; j <= n ; j++)
    {
      LU[i][j] = A[i][j];
    }
  }

  for (int k = 0; k <= n; k++)
  {
    /*
    * In finding the permutation matrix p that avoids dividing by zero
     * we take a slightly different approach.  For numerical stability
     * We find the element with the largest 
     * absolute value of those in the current first column (column k).  If all elements in
     * the current first column are zero then the matrix is singluar and throw an
     * error.
     * */
    p = 0;
    for (int i = k; i <= n; i++)
    {
      if (fabs(LU[i][k]) > p)
      {
        p = fabs(LU[i][k]);
        kp = i;
      }
    }
    if (p == 0)
    {
      printf("Error : singular matrix");
    }
    /*
    * These lines update the pivot array (which represents the pivot matrix)
     * by exchanging pi[k] and pi[kp].
     * */
    pik = pi[k];
    pikp = pi[kp];
    pi[k] = pikp;
    pi[kp] = pik;

    /*
    * Exchange rows k and kpi as determined by the pivot
     * */
    for (int i = 0; i <= n; i++)
    {
      aki = LU[k][i];
      akpi = LU[kp][i];
      LU[k][i] = akpi;
      LU[kp][i] = aki;
    }

    /*
    * Compute the Schur complement
     * */
    for (int i = k + 1; i <= n; i++)
    {
      LU[i][k] = LU[i][k] / LU[k][k];
      for (int j = k + 1; j <= n; j++)
      {
        LU[i][j] = LU[i][j] - (LU[i][k] * LU[k][j]);
      }
    }
  }
}


/////////////////////////////////////////////////////////////////////////////
// Awaits ASCII string "next" to keep going with the calibration process
boolean WaitSerialNext(void)
{
  if(GrabSerialMessage())
  {
    if(!strncmp("next", StringBuffer, 4))
      return(true);
  }

  // Add OSC input there to allow remote calibration without USB

  return(false);
}	







