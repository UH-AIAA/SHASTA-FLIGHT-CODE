/*
 * SRAD Avionics Flight Software
 *
 * O. Rangel Morales
 * (GitHub: OrlandoR4)
 * MIT license, all text above must be included in any redistribution
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <Orientation.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSO32.h>

// Timing
int32_t mstime, previoustime, deltatime;

int32_t previoustime_state, deltatime_state = 50;

int32_t deltatime_logslow = 200, deltatime_logfast = 20;
int32_t previoustime_log, deltatime_log = deltatime_logslow;

int32_t previoustime_radio, deltatime_radio = 500;

// Sensors
const uint32_t PIN_BUZZER = 33;
const uint32_t PIN_LED = 29;
const uint32_t LSM_CS = 40;
const uint32_t BMP_CS = 41;
const uint32_t ADXL_CS = 39;

Adafruit_LSM6DSO32 LSM;
Adafruit_BMP3XX BMP;
Adafruit_ADXL375 ADXL(ADXL_CS, &SPI, 12345);
Adafruit_BNO055 BNO(55, 0x28, &Wire);

Adafruit_GPS GPS(&Serial2);

// Sensor raw data
Vector3 lsm_gyro, lsm_acc;
Vector3 adxl_acc;

Vector3 bno_gyro, bno_acc, bno_mag;
Vector3 bno_euler;
Quaternion bno_orientation;

// Inertia data
Vector3 angular_rate;
Vector3 acceleration_body, acceleration_inertial;
Vector3 magnetic_flux;

Vector3 euler_angles;
Quaternion orientation;

// Temperatures
float lsm_temp, adxl_temp, bno_temp, bmp_temp;

// Barometer data
float bmp_press, bmp_alt;
float off_alt, prev_alt, v_vel;

// Calibration
bool rates_offset_set = false;
Vector3 rates_offset;

// States
enum class States{
  IdleNoCal, 
  Idle,
  Flight, 
  Landed
} flight_state;

// Thresholds
#define accel_liftoff_threshold                    30  // METERS PER SECOND^2
#define accel_liftoff_time_threshold              250  // MILLISECONDS
#define land_time_threshold                     30000  // MILLISECONDS
#define land_altitude_threshold                    50  // METERS

bool detect_liftoff(uint32_t dt, float meas_accel){
  static uint32_t liftoff_timer;

  if(meas_accel > accel_liftoff_threshold){
    liftoff_timer += dt;

    if(liftoff_timer > accel_liftoff_time_threshold){
      return true;
    }
  }else{
    liftoff_timer = 0;
  }

  return false;
}
bool detect_landing(uint32_t dt, float meas_alt){
  static uint32_t landing_timer;

  if(meas_alt < land_altitude_threshold){
    landing_timer += dt;
  }else{
    landing_timer = 0;
  }

  if(landing_timer > land_time_threshold){
    return true;
  }

  return false;
}

// Calibration
bool rates_offset_calibration(uint32_t dt, uint32_t sample_runtime){
  static uint32_t sampling_time = 0;
  static Vector3 pre_rates_offset;

  if((sampling_time <= sample_runtime) && !rates_offset_set){
    sampling_time += dt;
    rates_offset_set = false;

    if(angular_rate.get_magnitude() < 5.f * DEG_TO_RAD)
      pre_rates_offset += angular_rate * dt;
  }else{
    rates_offset_set = true;
  }

  if(rates_offset_set){
    rates_offset = pre_rates_offset/float(sample_runtime);
    return true;
  }else{
    return false;
  }
}

// Logging

bool log_enable = true;
File csvfile;

const String data_header = 
"time,lat,lon,"

"sat,spd,cou,g_alt,"

"state,"

"eul_x,eul_y,eul_z,"
"q_w,q_x,q_y,q_z,"
"q_wn,q_xn,q_yn,q_zn,"
"acc_i_x,acc_i_y,acc_i_z,"

"rate_x,rate_y,rate_z,"
"acc_b_x,acc_b_y,acc_b_z,"

"rate_xn,rate_yn,rate_zn,"
"acc_b_xn,acc_b_yn,acc_b_zn,"

"acc_b_xh,acc_b_yh,acc_b_zh,"

"press,alt,vel_z,"
"t_lsm,t_axl,t_bno,t_bmp"
;

// Misc.
void set_buzzer_LED_pulse(uint32_t dt, uint32_t on_time, uint32_t off_time, uint32_t frequency = 2000){
  static uint32_t pulse_timer;

  pulse_timer += dt;

  if(pulse_timer >= (on_time + off_time)){
    pulse_timer = 0;
    tone(PIN_BUZZER, frequency, on_time);
  }
  else if(pulse_timer < on_time)
  {
    digitalWrite(PIN_LED, 1);
  }
  else
  {
    digitalWrite(PIN_LED, 0);
  }
}

// Sensor read functions
bool read_LSM()
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  if(!LSM.getEvent(&accel, &gyro, &temp))
  {
    return false;
  }

  lsm_gyro.x = gyro.gyro.x;
  lsm_gyro.y = gyro.gyro.y;
  lsm_gyro.z = gyro.gyro.z;

  lsm_acc.x = accel.acceleration.x;
  lsm_acc.y = accel.acceleration.y;
  lsm_acc.z = accel.acceleration.z;

  lsm_temp = float(temp.temperature);

  return true;
}
bool read_BMP()
{
  if(!BMP.performReading())
  {
    return false;
  }

  bmp_temp = BMP.temperature;
  bmp_press = BMP.pressure;
  bmp_alt = BMP.readAltitude(1013.25) - off_alt;
  
  return true;
}
bool read_ADXL()
{
  sensors_event_t event;
  if(!ADXL.getEvent(&event))
  {
    return false;
  }

  adxl_acc.x = event.acceleration.x;
  adxl_acc.y = event.acceleration.y;
  adxl_acc.z = event.acceleration.z;

  adxl_temp = float(event.temperature);

  return true;
}
bool read_BNO()
{
  sensors_event_t orientationData , angVelocityData, magnetometerData, accelerometerData;

  if(!BNO.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER))
  {
    return false;
  }
  if(!BNO.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE))
  {
    return false;
  }
  if(!BNO.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER))
  {
    return false;
  }
  if(!BNO.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER))
  {
    return false;
  }

  imu::Quaternion quat = BNO.getQuat();
  bno_orientation.w = quat.w();
  bno_orientation.x = quat.x();
  bno_orientation.y = quat.y();
  bno_orientation.z = quat.z();
  //bno_orientation *= Quaternion().euler_to_quaternion(Vector3(0, 0, (90 + 20) * DEG_TO_RAD));

  bno_euler.x = -orientationData.orientation.heading;
  bno_euler.y = -orientationData.orientation.pitch;
  bno_euler.z = -orientationData.orientation.roll + 180.0;

  bno_gyro.x = angVelocityData.gyro.x;
  bno_gyro.y = angVelocityData.gyro.y;
  bno_gyro.z = angVelocityData.gyro.z;

  bno_acc.x = accelerometerData.acceleration.x;
  bno_acc.y = accelerometerData.acceleration.y;
  bno_acc.z = accelerometerData.acceleration.z;

  bno_mag.x = magnetometerData.magnetic.x;
  bno_mag.y = magnetometerData.magnetic.y;
  bno_mag.z = magnetometerData.magnetic.z;

  bno_temp = float(BNO.getTemp());

  return true;
}

void setup() 
{
  // Pin configurations
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  // USB Serial Port
  Serial.begin(115200);
  // Radio Serial Port
  Serial1.begin(9600);
  // GPS Serial Port
  Serial2.begin(115200);

  // Begin LSM6DSO32
  while(!LSM.begin_SPI(LSM_CS, &SPI))
  {
    Serial.println(F("LSM6DSO32 not found..."));
    delay(1000);
  }
  Serial.println(F("LSM6DSO32 intialized"));
  // Begin BMP390
  while(!BMP.begin_SPI(BMP_CS, &SPI))
  {
    Serial.println(F("BMP390 not found..."));
    delay(1000);
  }
  Serial.println(F("BMP390 intialized"));
  // Begin ADXL375
  while(!ADXL.begin())
  {
    Serial.println(F("ADXL375 not found..."));
    delay(1000);
  }
  Serial.println(F("ADXL375 intialized"));
  // Begin BNO055
  while(!BNO.begin())
  {
    Serial.println(F("BNO055 not found..."));
    delay(1000);
  }
  Serial.println(F("BNO055 intialized"));
  // Begin SD
  while(!SD.begin(BUILTIN_SDCARD)) {
    Serial.println(F("SD not found..."));
    delay(1000);
  }
  Serial.println(F("SD intialized"));

  // Configure LSM6DSO32
  LSM.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
  LSM.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  LSM.setAccelDataRate(LSM6DS_RATE_416_HZ);
  LSM.setGyroDataRate(LSM6DS_RATE_416_HZ);

  // Configure BMP390
  BMP.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
  BMP.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  BMP.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  BMP.setOutputDataRate(BMP3_ODR_200_HZ);

  // Configure ADXL
  ADXL.setDataRate(ADXL343_DATARATE_200_HZ);

  // Configure BNO055
  BNO.setMode(OPERATION_MODE_CONFIG);
  BNO.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);
  BNO.setMode(OPERATION_MODE_NDOF);

  // GPS Initialization and Configuration
  GPS.begin(115200);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  // Datalogging init

  //Create datalogging file
  char csvfilename[17] = "FL0.csv";
  for(uint32_t i = 0; SD.exists(csvfilename); i++){
    sprintf(csvfilename, "FL%d.csv", i);
  }

  //Open file for writing
  csvfile = SD.open(csvfilename, FILE_WRITE);

  //Print data header
  csvfile.println(data_header);
  csvfile.flush();
}

//#define SERIALDEBUG

void loop()
{
  previoustime = mstime;
  mstime = millis();
  deltatime = mstime - previoustime;
  float deltatime_s = float(deltatime) * 0.001;

  // Sensors Read
  read_ADXL();
  read_BMP();
  read_BNO();
  read_LSM();

  // GPS Update
  while(GPS.available())
  {
    GPS.read();
  }

  if(GPS.newNMEAreceived())
  {
    GPS.parse(GPS.lastNMEA());
  }
  
  // Orientation update
  angular_rate = lsm_gyro - rates_offset;
  acceleration_body = lsm_acc;
  magnetic_flux = bno_mag;

  acceleration_inertial = orientation.rotate_vector(acceleration_body);

  orientation.update_with_rates(deltatime_s, angular_rate);
  
  if (flight_state < States::Flight)
  {
    orientation.update_with_accel(acceleration_inertial, Vector3(0, 0, 1), 0.1);
    orientation.update_with_mag(bno_mag, lsm_acc, Vector3(0, -1, 0), Quaternion().euler_to_quaternion(Vector3(0, 0, 14 * DEG_TO_RAD)), 0.1);
  }

  acceleration_inertial.z -= 9.8066f;
  euler_angles = orientation.quaternion_to_euler().rad_to_deg();

  // Velocity calculation
  if (deltatime_s > 0)
  {
    v_vel = (bmp_alt - prev_alt) / deltatime_s;
  }
  prev_alt = bmp_alt;
  
  // STATE UPDATE
  if (mstime - previoustime_state >= deltatime_state)
  {
    previoustime_state = mstime;

    switch (flight_state)
    {
      //UNCALIBRATED IDLE
      case (States::IdleNoCal):
      {
        //INDICATION
        set_buzzer_LED_pulse(deltatime, 250, 2000);

        //LIFTOFF DETECTION
        if (detect_liftoff(deltatime_state, acceleration_body.z))
        {
          deltatime_log = deltatime_logfast;
          flight_state = States::Flight;
          off_alt = bmp_alt;

          break;
        }

        //Transition on IMU calibration
        rates_offset_calibration(deltatime_state, 5000);
        if(rates_offset_set){
          flight_state = States::Idle;
        }

        break;
      }
      //IDLE
      case (States::Idle):
      {
        //INDICATION
        set_buzzer_LED_pulse(deltatime, 1000, 3000);

        //LIFTOFF DETECTION
        if (detect_liftoff(deltatime_state, acceleration_body.z))
        { 
          deltatime_log = deltatime_logfast;
          flight_state = States::Flight;
          off_alt = bmp_alt;
          tone(PIN_BUZZER, 2000, 2000);

          break;
        }

        break;
      }
      //FLIGHT
      case (States::Flight):
      {
        if(detect_landing(deltatime_state, bmp_alt)){ 
          flight_state = States::Landed;
        }
        break;
      }
      //LANDED
      case (States::Landed):
      {
        log_enable = false;
        set_buzzer_LED_pulse(deltatime, 1000, 500);
        break;
      }

      default: 
        break;
    }
  }

  // RADIO UPDATE
  if (mstime - previoustime_radio >= deltatime_radio)
  {
    previoustime_radio = mstime;

    Serial1.print(F("ALT: ")); 
    Serial1.print(bmp_alt, 1); 
    Serial1.print(F("\t"));
    
    Serial1.print(F("VEL: ")); 
    Serial1.print(v_vel, 1);   
    Serial1.print(F("\t"));

    Serial1.print(F("LAT: "));
    Serial1.print(GPS.latitudeDegrees, 6); 
    Serial1.print(GPS.lat); 
    Serial1.print(F("\t"));

    Serial1.print(F("LON: "));
    Serial1.print(GPS.longitudeDegrees, 6); 
    Serial1.print(GPS.lon); 
    Serial1.print(F("\t"));

    Serial1.print(F("STN: "));
    Serial1.print((int)GPS.satellites); 
    Serial1.print(F("\t"));

    Serial1.print(F("STA: "));            
    Serial1.print(F("\t"));

    Serial1.print(uint32_t(flight_state));
    Serial1.print(F("\n"));
  }

  // LOG UPDATE
  if (mstime - previoustime_log >= deltatime_log)
  {
    previoustime_log = mstime;

    if (log_enable)
    {
      csvfile.print(mstime);                  csvfile.print(",");

      csvfile.print(GPS.latitudeDegrees, 6); 
      csvfile.print(GPS.lat);                 csvfile.print(",");

      csvfile.print(GPS.longitudeDegrees, 6); 
      csvfile.print(GPS.lon);                 csvfile.print(",");

      csvfile.print((int32_t)GPS.satellites); csvfile.print(",");
      csvfile.print(GPS.speed, 3);            csvfile.print(",");
      csvfile.print(GPS.angle, 3);            csvfile.print(",");
      csvfile.print(GPS.altitude, 3);         csvfile.print(",");

      csvfile.flush();

      csvfile.print(int32_t(flight_state));   csvfile.print(",");

      csvfile.print(euler_angles.x, 2);       csvfile.print(",");
      csvfile.print(euler_angles.y, 2);       csvfile.print(",");
      csvfile.print(euler_angles.z, 2);       csvfile.print(",");

      csvfile.print(orientation.w, 5);        csvfile.print(",");
      csvfile.print(orientation.x, 5);        csvfile.print(",");
      csvfile.print(orientation.y, 5);        csvfile.print(",");
      csvfile.print(orientation.z, 5);        csvfile.print(",");

      csvfile.print(bno_orientation.w, 5);    csvfile.print(",");
      csvfile.print(bno_orientation.x, 5);    csvfile.print(",");
      csvfile.print(bno_orientation.y, 5);    csvfile.print(",");
      csvfile.print(bno_orientation.z, 5);    csvfile.print(",");

      csvfile.flush();

      csvfile.print(acceleration_inertial.x, 4);       csvfile.print(",");
      csvfile.print(acceleration_inertial.y, 4);       csvfile.print(",");
      csvfile.print(acceleration_inertial.z, 4);       csvfile.print(",");

      csvfile.print(angular_rate.x, 4);       csvfile.print(",");
      csvfile.print(angular_rate.y, 4);       csvfile.print(",");
      csvfile.print(angular_rate.z, 4);       csvfile.print(",");

      csvfile.print(acceleration_body.x, 4);       csvfile.print(",");
      csvfile.print(acceleration_body.y, 4);       csvfile.print(",");
      csvfile.print(acceleration_body.z, 4);       csvfile.print(",");

      csvfile.print(bno_gyro.x, 4);       csvfile.print(",");
      csvfile.print(bno_gyro.y, 4);       csvfile.print(",");
      csvfile.print(bno_gyro.z, 4);       csvfile.print(",");

      csvfile.print(bno_acc.x, 4);       csvfile.print(",");
      csvfile.print(bno_acc.y, 4);       csvfile.print(",");
      csvfile.print(bno_acc.z, 4);       csvfile.print(",");

      csvfile.print(adxl_acc.x, 2);       csvfile.print(",");
      csvfile.print(adxl_acc.y, 2);       csvfile.print(",");
      csvfile.print(adxl_acc.z, 2);       csvfile.print(",");

      csvfile.flush();

      csvfile.print(bmp_press, 6);    csvfile.print(",");
      csvfile.print(bmp_alt, 4);      csvfile.print(",");
      csvfile.print(v_vel, 4);        csvfile.print(",");

      csvfile.print(lsm_temp, 2);       csvfile.print(",");
      csvfile.print(adxl_temp, 2);      csvfile.print(",");
      csvfile.print(bno_temp, 2);       csvfile.print(",");
      csvfile.print(bmp_temp, 2);       csvfile.print(",");

      csvfile.println();
      csvfile.flush();
    }

    #ifdef SERIALDEBUG
      Serial.print(F("XYZ: "));
      Serial.print(euler_angles.x, 1); Serial.print(F("\t"));
      Serial.print(euler_angles.y, 1); Serial.print(F("\t"));
      Serial.print(euler_angles.z, 1); Serial.print(F("\t"));

      Serial.print(F("ALT: "));
      Serial.print(bmp_alt, 1);   Serial.print(F("\t"));

      Serial.print(F("VEL: "));
      Serial.print(v_vel, 1);   Serial.print(F("\t"));

      Serial.print(F("LAT: "));
      Serial.print(GPS.latitudeDegrees, 6); 
      Serial.print(GPS.lat); 
      Serial.print(F("\t"));

      Serial.print(F("LON: "));
      Serial.print(GPS.longitudeDegrees, 6); 
      Serial.print(GPS.lon); 
      Serial.print(F("\t"));

      /*Serial.print(F("COU: "));
      Serial.print(GPS.speed, 1); 
      Serial.print(F(" > "));
      Serial.print(GPS.angle);
      Serial.print(F("\t"));*/

      Serial.print(F("SAT: "));
      Serial.print((int)GPS.satellites); Serial.print(F("\t"));

      Serial.print(F("STATE: "));                 Serial.print(F("\t"));
      Serial.print(uint32_t(flight_state));       Serial.print(F("\n"));
    #endif

    Serial.print(F("RATE: "));              Serial.print(F("\t"));
    Serial.print(1000.0/float(deltatime));  Serial.print(F("\t"));
  }
}