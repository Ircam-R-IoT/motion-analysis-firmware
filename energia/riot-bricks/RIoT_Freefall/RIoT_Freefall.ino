/* R-IoT :
 Texas Instrument CC3200 Internet of Things / Sensor hub / Dev Platform
 80 MHz 32 Bit ARM MCU + Wifi stack / modem
 
 IRCAM - Emmanuel FLETY - Music Bricks
 
 Battery voltage : protection voltage divider of 0.364. ADC vRef (and max input) = 1.7V (ish)
 Battery reading in volt = [analogRead() * 1.7] / (4096 * 0.364)
 or, simplified to 
 Battery reading in volt = analogRead() / 877.
 
 BASIC DEMO TEST : grabs the raw data from the 9 axis IMU, obtain EULER angles via
 Madgwick algorithm and exports everything via OSC with fixed port and destination
 computer IP, and prints out all data on the serial port
 
 */

#include <stdio.h>
#include <strings.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

#include "common.h"
#include "LSM9DS0.h"
#include "osc.h"
#include "features.h"

/////////////////////////////////////////////////////////////
// Global vars
byte mac[6];
char ssid[32] = "riot";
IPAddress DestIP = IPAddress(192,168,1,100);
uint16_t DestPort = 8888;
uint8_t ModuleID = 0;
unsigned long SampleRate = 5;
int status = WL_IDLE_STATUS;
int statusAP = false;
int PacketStatus;
int TempInt = 0;

WiFiUDP UdpPacket;
WiFiUDP ConfigPacket;
OscBuffer RawSensors;
OscBuffer Quaternions;
OscBuffer EulerAngles;
OscBuffer Freefall;

unsigned long ElapsedTime = 0;
unsigned long ElapsedTime2 = 0;

////////////////////////////////////////////////////////////
// Sensor storage
short unsigned int BatteryVoltage = 0, SwitchState;
Word AccelerationX, AccelerationY, AccelerationZ;
Word GyroscopeX, GyroscopeY, GyroscopeZ;
Word MagnetometerX, MagnetometerY, MagnetometerZ;
Word Temperature;

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
#define zeta  0.0f
#define beta 0.4f    // Much faster - noisier
//#define beta 0.041f    // 10 seconds or more to have the quaternion converge to the right orientation - super smooth

float pitch, yaw, roll, heading;
float deltat = 0.005f;        // integration interval for both filter schemes - 5ms by default

// Accel bias can be obtained if a super flat surface is available otherwise leave as zero, worse case it's just a small
// offset on the absolute zero for pitch and roll. The only important offset is the gyro Z one to avoid drifting on the YAW.
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

// For the object "freefall"
float acc_norm;
float gyr_last_three[3][3] = {{0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};// g_x, g_y, g_z at times t_n, t_n-1, t_n-2
float gyr_delta[3];
float gyr_norm, gyr_delta_norm;
unsigned long FallBegin = 0, FallEnd = 0;
float fallDuration, isFalling = 0.0f;
unsigned short int LoopIndex = 0;
unsigned short int LoopIndexPeriod = 3;

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
// temporary strings
char StringBuffer[MAX_STRING];

void setup() {
  // Basic I/Os
  pinMode(POWER_LED, OUTPUT);
  pinMode(SWITCH_INPUT, INPUT_PULLUP);

  // POWER On indicator  
  digitalWrite(POWER_LED, HIGH);

  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  Serial.println("R-IoT free fall detection demo");

  digitalWrite(POWER_LED, HIGH);

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
  // Accelerometers are using the whole dynamic range (16 bits), gyro / mag aren't
  gRes = 2000.0 / 28571.4; 	// +- 2000 deg/s
  aRes = 8.0 / 32768.0;         // +- 8g
  mRes = 2.0 / 25000.0;         // +- 2 gauss

  // Finalize the bias unit conversion
  for(int i = 0 ; i < 3 ; i++)
  {
    gbias[i] = gRes * (float)gyro_bias[i];
    abias[i] = aRes * (float)accel_bias[i];
    mbias[i] = mRes * (float)mag_bias[i];
  }

  // attempt to connect to Wifi network:
  Serial.print("R-IoT connecting to: ");
  // print the network name (SSID);
  Serial.println(ssid); 

  digitalWrite(POWER_LED, LOW);
  // Start the Wifi connection & config
  WiFi.begin(ssid); 
  WiFi.macAddress(mac);

  // Prep the UDP packet
  UdpPacket.begin(DestPort);
  UdpPacket.beginPacket(DestIP, DestPort);

  // Prepare the OSC message structure
  sprintf(StringBuffer, "/%u/raw\0",ModuleID);
  PrepareOSC(&RawSensors, StringBuffer, 'i', 12);    // 11 integers + temperature
  sprintf(StringBuffer, "/%u/quat\0",ModuleID);
  PrepareOSC(&Quaternions, StringBuffer, 'f', 4);    // 4 floats
  sprintf(StringBuffer, "/%u/euler\0",ModuleID);
  PrepareOSC(&EulerAngles, StringBuffer, 'f', 4);    // 4 floats (yaw, pitch, roll, heading)
  sprintf(StringBuffer, "/%u/freefall\0", ModuleID);
  PrepareOSC(&Freefall, StringBuffer, 'f', 3);          // 3 floats: acceleration magnitude, boolean isFalling, freefall duration
  
  ElapsedTime = millis();
  ElapsedTime2 = millis();
}



void loop() {

  if(WiFi.status() != WL_CONNECTED)
  {
    if((millis() - ElapsedTime2) > 300)
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

    // If reaching the sampling period, let's grab new sensor data and process it
    if((millis() - ElapsedTime >= SampleRate) && (WiFi.status() == WL_CONNECTED))
    {       
      ElapsedTime = millis();
      digitalWrite(POWER_LED, HIGH);
      // read the battery status
      BatteryVoltage = analogRead(BATT_MONITORING);
      SwitchState = digitalRead(SWITCH_INPUT);

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
      
      //FREEFALL
      acc_norm = magnitude3D(a_x,a_y,a_z);
      gyr_norm = magnitude3D(g_x,g_y,g_z);
      
      gyr_last_three[0][LoopIndex % 3] = g_x;
      gyr_last_three[1][LoopIndex % 3] = g_y;
      gyr_last_three[2][LoopIndex % 3] = g_z;
      
      gyr_delta[0] = delta(gyr_last_three[0][(LoopIndex+1) % 3], g_x, 1.0f); //last argument should be deltat for a result in deg/s2
      gyr_delta[1] = delta(gyr_last_three[1][(LoopIndex+1) % 3], g_y, 1.0f);
      gyr_delta[2] = delta(gyr_last_three[2][(LoopIndex+1) % 3], g_z, 1.0f);
      
      gyr_delta_norm = magnitude3D(gyr_delta[0], gyr_delta[1], gyr_delta[2]);
      
      if (acc_norm < FREEFALL_ACC_THRESHOLD || (gyr_norm > FREEFALL_GYR_THRESHOLD && gyr_delta_norm < FREEFALL_GYR_DELTA_THRESHOLD)){
        //Falling state detected
        if (isFalling == 0.0f){
          //Beginning of a fall
          isFalling = 1.0f;
          FallBegin = millis();
        }
        FallEnd = millis();
      }
      else {
        //Not falling
        if (isFalling == 1.0f){
          //End of a fall
          isFalling = 0.0f;
        }
      }
      fallDuration = (float) (FallEnd-FallBegin);

      LoopIndex = (LoopIndex + 1) % LoopIndexPeriod;

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
      
      pData = Freefall.pData;
      FloatToBigEndian(pData, &acc_norm);
      pData += sizeof(float);
      FloatToBigEndian(pData, &isFalling);
      pData += sizeof(float);
      FloatToBigEndian(pData, &fallDuration);

      UdpPacket.write((uint8_t*)RawSensors.buf, RawSensors.PacketSize);
      UdpPacket.endPacket();
      UdpPacket.write((uint8_t*)Quaternions.buf, Quaternions.PacketSize);
      UdpPacket.endPacket();
      // Optional
      UdpPacket.write((uint8_t*)EulerAngles.buf, EulerAngles.PacketSize);
      UdpPacket.endPacket();
      UdpPacket.write((uint8_t*)Freefall.buf, Freefall.PacketSize);
      UdpPacket.endPacket();
      
      digitalWrite(POWER_LED, LOW);
    }
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


////////////////////////////////////////////////////////////////////////////////////
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




