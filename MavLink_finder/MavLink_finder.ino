//Includes
#include "ardupilotmega/mavlink.h"
#include <HardwareSerial.h>
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <Math.h>
#include <Wire.h>
#include <MMC34160PJ.h>
#include <Servo.h>

//Defines
#define deg2rad(deg) ((deg) * M_PI / 180.0)
#define rad2deg(rad) ((rad) * 180.0 / M_PI)
#define I2C_ADDRESS 0x30
#define clockwise 85
#define anticlockwise 100



//Variables
long uav_lat, uav_lng, uav_alt, g_lat, g_lng, g_alt; //
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;
static const uint32_t GPSBaud = 9600;
static const int RXPin = PB11, TXPin = PB10;

//Objects
HardwareSerial mavdata(PA3,PA2);
HardwareSerial gpss(RXPin,TXPin); // compiles, can also use (USART2)
TinyGPSPlus gps;
MMC34160PJ magnetometer(I2C_ADDRESS);
Servo yaw_servo;
Servo pitch_servo;

//setup
void setup()
{
  Serial.begin(115200);
  mavdata.begin(115200);
  gpss.begin(9600);
  pitch_servo.attach(PB8);
  yaw_servo.attach(PB9);
  Wire.begin();
  Wire.setClock(1000);
  if(!magnetometer.init()){
    Serial.println("MMC34160PJ not connected!");
  }
}

void loop() 
{
  heartbeat();
  float mag_angle = 0;
  if(magnetometer.readData()){
    mag_angle = magnetometer.getAngle();
  } 
  else 
  {
    magnetometer.init();
  }
  delay(200);
  comm_receive();
  smartDelay(200);
  update_gps();
  double dist=haversine_Distance(g_lat,g_lng,uav_lat,uav_lng);
  double angle_yaw=calculate_bearing(g_lat,g_lng,uav_lat,uav_lng);
  double angle_pitch=find_Angle(dist, (uav_alt-g_alt));
  float angle_to_see=mag_angle-angle_yaw;
  pitch_servo.write(angle_pitch);   //Updating pitch servo
  while(true)
  {
    if(angle_to_see <= -1.5)
    {
      yaw_servo.write(clockwise);
    }
    else if(angle_to_see >= 1.5)
    {
      yaw_servo.write(anticlockwise);
    }
    else
    {
      yaw_servo.write(90);
      break;
    }
  }
//  print_all(dist,angle_yaw,angle_pitch,mag_angle);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpss.available())
    {
      gps.encode(gpss.read());
    }
  } while (millis() - start < ms);
}

//void print_all(double dist,double angle_yaw,double angle_pitch,float mag_angle)
//{
//  Serial.print("uav lat= ");
//  Serial.print(uav_lat);
//  Serial.print(" uav lon=");
//  Serial.print(uav_lng);
//  Serial.print(" uav alt=");
//  Serial.print(uav_alt);
//  Serial.print(" gps lat= ");
//  Serial.print(g_lat);
//  Serial.print(" gps lon=");
//  Serial.print(g_lng);
//  Serial.print(" gps alt=");
//  Serial.print(g_alt);
//  Serial.print("  dist=");
//  Serial.print(dist);
//  Serial.print("  yaw=");
//  Serial.print(angle_yaw);
//  Serial.print("  pitch=");
//  Serial.print(angle_pitch);
//  Serial.print("  mag=");
//  Serial.print(mag_angle);
//  float angle_to_see=mag_angle-angle_yaw;
//  Serial.print("  See=");
//  Serial.println(angle_to_see);
//}

void update_gps()
{
  g_lat= gps.location.lat()*1e7;
  g_lng= gps.location.lng()*1e7;
  g_alt = gps.altitude.meters()*1e3;
}

void heartbeat()
{
//Serial.println("MAVLink starting.");
  int sysid = 1;                   
  int compid = 158;                ///< The component sending the message
  int type = MAV_TYPE_ANTENNA_TRACKER;   ///< This system is an Antenna Tracker
 
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
 
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    previousMillisMAVLink = currentMillisMAVLink;

    mavdata.write(buf,len);
    num_hbs_pasados++;
    if(num_hbs_pasados>=num_hbs) {
      Serial.println("Streams requested!");
      Mav_Request_Data();
      num_hbs_pasados=0;
    }

  }
}

void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   */

  // To be setup according to the needed information to be requested from the Pixhawk
  //const int  maxStreams = 1;
  const uint16_t MAVRate = 0x05;
    /*
     * mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, 
     *    mavlink_message_t* msg,
     *    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, 
     *    uint16_t req_message_rate, uint8_t start_stop)
     */
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAV_DATA_STREAM_POSITION, MAVRate, 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    bool wr=mavdata.write(buf, len);
    Serial.println(wr);
}


void comm_receive() 
{
  mavlink_message_t msg;
  mavlink_status_t status;
  while(mavdata.available() > 0) 
  {
    uint8_t c = mavdata.read();
    // Try to get a new message
//    bool val=mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status);
//    if(val)
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) 
    {
      // Handle message
      switch(msg.msgid) {

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:  // #30
          {
            mavlink_global_position_int_t locate;
            mavlink_msg_global_position_int_decode(&msg, &locate);

            uav_alt = locate.alt;
            uav_lat = locate.lat ;
            uav_lng = locate.lon ;
            previousMillisMAVLink = millis();
          }
          break;
          
       default:
          break;
      }
    }
  }
}

double haversine_Distance(long lat1, long lon1, long lat2, long lon2)
{
  double R = 6371e3; // Earth's radius in meters
  double lat1_rad = deg2rad(lat1 / 1e7);
  double lat2_rad = deg2rad(lat2 / 1e7);
  double delta_lat_rad = deg2rad((lat2 - lat1) / 1e7);
  double delta_lon_rad = deg2rad((lon2 - lon1) / 1e7);

  double a = sin(delta_lat_rad / 2) * sin(delta_lat_rad / 2) +
             cos(lat1_rad) * cos(lat2_rad) *
             sin(delta_lon_rad / 2) * sin(delta_lon_rad / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  double distance = R * c;
  return distance;
}


double calculate_bearing(long lat1, long lon1, long lat2, long lon2)
{
  double dLon = deg2rad((lon2 - lon1) / 1e7);
  double lat1_rad = deg2rad(lat1 / 1e7);
  double lat2_rad = deg2rad(lat2 / 1e7);
  double y = sin(dLon) * cos(lat2_rad);
  double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dLon);
  double bearing = atan2(y, x);
  return fmod((rad2deg(bearing) + 360) , 360);
}

double find_Angle(double distance, double height) 
{
    height = height / 1000; //convert height from mm to meters
    double angle = atan2(height, distance) * (180 / M_PI);
    return angle;
}
