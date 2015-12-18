// Angle calculations and control part is running at 200Hz from DMP solution
// DMP is using the gyro_bias_no_motion correction method.
// The board needs at least 10-15 seconds to give good values...

// STEPPER MOTOR PINS
// ENABLE PIN: D4
// Motor 1 LEFT
// STEP Motor1: D6 -> PORTD,6 (Mega: PORTH,3) (MultiWii: 5, PORTD,5)
// DIR  Motor1: D7 -> PORTB,0 (Mega: PORTH,4) (MultiWii: 6, PORTD,6)
// STEP Motor2: D8-> PORTB,0 (Mega: PORTH,5) (MultiWii: 7, PORTD,7)
// DIR  Motor2: D9-> PORTB,1 (Mega: PORTB,6) (MultiWii: 8, PORTB,0)
// To control the stepper motors we use Timer1 interrupt running at 25Khz. We control the speed of the motors
// Buzzer A1
// Note: We could not use WITA led because pin 13 is already used for stepper motor control
// Robot servo arm connected to D5 (MultiWii: 3)
// Distance sensor (sonar) connected to A2 & A3 (MultiWii D3 and A5)
// Battery monitor (voltage divider) connected to A0
// We use a standard PID control for robot stability
// We have a P control for speed control and a PD control for stability (robot angle)
// 		The output of the control (motor speed) is integrated so it?s really an acceleration

// We control the robot from a WIFI module using OSC standard UDP messages (OSCmini library)
//    fadder1: Throttle   from 0 to 1 with 0.5 middle
//    fadder2: Steering from -512 to +512
//    arm: Move arm (and robot raiseup)

//    toggle1: autonomous mode


// Robot autonomous mode
//   the robot start walking until reach an obstacle. When find an obstacle, start steering until it find a free way and continue walking


#include <Wire.h>
#include <I2Cdev.h>
#include <JJ_MPU6050_DMP_6Axis.h>  // Modified version of the library to work with DMP (see comments inside)
#include <SPI.h>
#include <RF24.h>
#include <DebugUtils.h>
#include "robotDefinitions.h"


#define DEBUG 0 // 0 laufen die Motoren, 1 kein Serial, 2 incl Serial
#define REMOTE 1
#define DMP_WAIT 15
#define LEFT 0
#define MIDDLE 1
#define RIGHT 2


#define SHUTDOWN_WHEN_BATTERY_OFF 0

#define buzzPin A1
#define ledPin 13

#define ZERO_SPEED 65535

#define MOTORCONSTRAIN 300

#define MAX_THROTTLE 400
#define MAX_STEERING 200
#define MAX_TARGET_ANGLE 12


//#define I2C_SPEED 100000L
#define I2C_SPEED 400000L
//#define I2C_SPEED 800000L

#define ACCEL_SCALE_G 8192             // (2G range) G = 8192
#define ACCEL_WEIGHT 0.01
#define GYRO_BIAS_WEIGHT 0.005

#define NEARUP 40
#define DOWN  70

// MPU6000 sensibility   (0.0609 => 1/16.4LSB/deg/s at 2000deg/s, 0.03048 1/32.8LSB/deg/s at 1000deg/s)
#define Gyro_Gain 0.03048
#define Gyro_Scaled(x) x*Gyro_Gain //Return the scaled gyro raw data in degrees per second

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

// Default control terms
#define KP 0.40 // 0.22
#define KD 50   // 30 28
#define KP_THROTTLE 0.065  //0.08
#define KI_THROTTLE 0.05


#define ITERM_MAX_ERROR 40   // Iterm windup constants
#define ITERM_MAX 5000


#define batteryPin A0
#define BATTERY_WARNING 7.0  // (7.0 volts) aprox
#define BATTERY_SHUTDOWN 6.8 // (6.8 volts)


RF24 radio(9, 10);
#define M1SPort PORTD
#define M1SPin 5
#define M1DPort PORTD
#define M1DPin 6

#define M2SPort PORTD
#define M2SPin 7
#define M2DPort PORTB
#define M2DPin 0

#define motor1StepPin 5
#define motor1DirPin 6
#define motor2StepPin 7
#define motor2DirPin 8

#define motorEnablePin 4

#define servoPin 3

#define DATARATE RF24_2MBPS
//#define DATARATE RF24_1MBPS
//#define DATARATE RF24_250KBPS

#define CHANNEL 20

#define AUTONOMOUS 1   //Operation Mode
#define MANUAL 0       //Operation Mode


// automom
const byte distCorr = 60;
const byte distCloseup = 30;
const byte throttleAutoMax = 100;
const byte autoSteering = 60;

#define WALK 1
#define STEER 2
#define TURN 8
#define BLOCKED 9


typedef struct
{
  int throttle;
  int steering;
  int Kd;
  int Kp;
  int Kp_Throttle;
  int Ki_Throttle;
  boolean button1;
  boolean button2;
  boolean button3;
  byte checksum;
}
controlDef;

controlDef controlPak;



typedef struct
{
  float throttle;
  float steering;
  float angle;
  int battery;
  float target;
  float Kd;
  float Kp;
  byte mode;
  int text_ref;
  int number;
}
displayDef;

displayDef displayPak;


uint8_t operationMode = 0; // 0: MANUAL MODE   1: autonomous MODE

byte distance[3];
int autStat;
unsigned long entrySteer = 0;



// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (for us 18 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[18]; // FIFO storage buffer
Quaternion q;

uint8_t remoteLoopCounter = 0;
uint8_t loop_counter;       // To generate a medium loop 40Hz
uint8_t slow_loop_counter;  // slow loop 2Hz
long timer_old;
long timer_value;
float dt;

// class default I2C address is 0x68
MPU6050 mpu;


float angle_adjusted;
float angle_adjusted_Old;

float Kp = KP;
float Kd = KD;
float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;
float Kp_user = KP;
float Kd_user = KD;
float Kp_thr_user = KP_THROTTLE;
float Ki_thr_user = KI_THROTTLE;
bool newControlParameters = false;
bool modifing_control_parameters = false;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
float steering;
float throttle;
float control_output;
int16_t speedActual[2];

int16_t speedRequired[2];

int16_t speed_m[2];           // Actual speed of motors
uint8_t dir_m[2];             // Actual direction of steppers motors
int16_t actual_robot_speed;          // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;          // overall robot speed (measured from steppers speed)
float estimated_speed_filtered;

unsigned int _average;

bool oldButton3 = 0;

// I2C
byte place;
int dist_min;
byte dir_min;

bool controlSleep = false;


int speedup = 10;

int16_t MAX_ACCEL = 7;

uint16_t counter_m[2];        // counters for periods
uint16_t period_m[2][8];      // Eight subperiods
uint8_t period_m_index[2];    // index for subperiods

float maxAngle = 0;

float userSteering = 0;
float userThrottle = 0;

bool fallen = false;

bool raiseUp = false;

bool exitFallen = false;

statusDef robotStat = up;

boolean firsttimeSteer = true;
int autonomSteer = 0;


boolean firsttimeTurn = true;
unsigned long	entryTime = 0;


// Topology NRF24L01
// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = {
  0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL
};


int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

// DMP FUNCTIONS
// This function defines the weight of the accel on the sensor fusion
// default value is 0x80
// The official invensense name is inv_key_0_96 (??)
void dmpSetSensorFusionAccelGain(uint8_t gain)
{
  // INV_KEY_0_96
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(gain);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);
}

// Quick calculation to obtein Phi angle from quaternion solution
float dmpGetPhi() {
  mpu.getFIFOBytes(fifoBuffer, 16); // We only read the quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.resetFIFO();  // We always reset FIFO

  //return( asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI); //roll
  return (atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * RAD2GRAD);
}

// PD implementation. DT is in miliseconds
float stabilityPDControl(float _DT, float _input, float _setPoint,  float _Kp, float _Kd)
{
  float _error;
  float _output;

  _error = _setPoint - _input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  _output = _Kp * _error + (_Kd * (_setPoint - setPointOld) - Kd * (_input - PID_errorOld2)) / _DT; // + error - PID_error_Old2
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  PID_errorOld2 = PID_errorOld;
  PID_errorOld = _input;  // error for Kd is only the input component
  setPointOld = _setPoint;
  return (_output);
}

// P control implementation.
float speedPControl(float _input, float _setPoint,  float _Kp)
{
  float _error;

  _error = _setPoint - _input;

  return (_Kp * _error);
}

// PI implementation. DT is in miliseconds
float speedPIControl(float _DT, float _input, float _setPoint,  float _Kp, float _Ki)
{
  float _error;
  float _output;

  _error = _setPoint - _input;
  PID_errorSum += constrain(_error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  _output = _Kp * _error + _Ki * PID_errorSum * _DT * 0.001;
  return (_output);
}



// 200ns => 4 instructions at 16Mhz
void delay_200ns()
{
  __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
}


ISR(TIMER1_COMPA_vect)
{
  counter_m[0]++;
  counter_m[1]++;
  if (counter_m[0] >= period_m[0][period_m_index[0]])
  {
    counter_m[0] = 0;
    if (period_m[0][0] == ZERO_SPEED)
      return;
    if (dir_m[0])
      SET(M1DPort, M1DPin); // DIR Motor 1
    else
      CLR(M1DPort, M1DPin);
    // We need to wait at lest 200ns to generate the Step pulse...
    period_m_index[0] = (period_m_index[0] + 1) & 0x07; // period_m_index from 0 to 7
    //delay_200ns();
    SET(M1SPort, M1SPin); // STEP Motor 1
    delayMicroseconds(1);
    CLR(M1SPort, M1SPin);

  }
  if (counter_m[1] >= period_m[1][period_m_index[1]])
  {
    counter_m[1] = 0;
    if (period_m[1][0] == ZERO_SPEED)
      return;
    if (dir_m[1])
      SET(M2DPort, M2DPin);  // DIR Motor 2
    else
      CLR(M2DPort, M2DPin);
    period_m_index[1] = (period_m_index[1] + 1) & 0x07;
    //delay_200ns();
    SET(M2SPort, M2SPin); // STEP Motor 1
    delayMicroseconds(1);
    CLR(M2SPort, M2SPin);
  }
}


// Dividimos en 8 subperiodos para aumentar la resolucion a velocidades altas (periodos peque?os)
// subperiod = ((1000 % vel)*8)/vel;
// Examples 4 subperiods:
// 1000/260 = 3.84  subperiod = 3
// 1000/240 = 4.16  subperiod = 0
// 1000/220 = 4.54  subperiod = 2
// 1000/300 = 3.33  subperiod = 1
void calculateSubperiods(uint8_t motor)
{
  int _subperiod;
  int _absSpeed;
  uint8_t j;

  if (speed_m[motor] == 0)
  {
    for (j = 0; j < 8; j++)
      period_m[motor][j] = ZERO_SPEED;
    return;
  }
  if (speed_m[motor] > 0 )   // Positive speed
  {
    dir_m[motor] = 1;
    _absSpeed = speed_m[motor];
  }
  else                       // Negative speed
  {
    dir_m[motor] = 0;
    _absSpeed = -speed_m[motor];
  }

  for (j = 0; j < 8; j++)
    period_m[motor][j] = 1000 / _absSpeed;
  // Calculate the subperiod. if module <0.25 => subperiod=0, if module < 0.5 => subperiod=1. if module < 0.75 subperiod=2 else subperiod=3
  _subperiod = ((1000 % _absSpeed) * 8) / _absSpeed; // Optimized code to calculate subperiod (integer math)
  if (_subperiod > 0)
    period_m[motor][1]++;
  if (_subperiod > 1)
    period_m[motor][5]++;
  if (_subperiod > 2)
    period_m[motor][3]++;
  if (_subperiod > 3)
    period_m[motor][7]++;
  if (_subperiod > 4)
    period_m[motor][0]++;
  if (_subperiod > 5)
    period_m[motor][4]++;
  if (_subperiod > 6)
    period_m[motor][2]++;
}


void setMotorSpeed(uint8_t _motor, int16_t _tspeed)
{
  // WE LIMIT MAX ACCELERATION
  if ((speed_m[_motor] - _tspeed) > MAX_ACCEL)
    speed_m[_motor] -= MAX_ACCEL;
  else if ((speed_m[_motor] - _tspeed) < -MAX_ACCEL)
    speed_m[_motor] += MAX_ACCEL;
  else
    speed_m[_motor] = _tspeed;

  calculateSubperiods(_motor);  // We use four subperiods to increase resolution

  // To save energy when its not running...
  if ((speed_m[0] == 0) && (speed_m[1] == 0))
  {
#if DEBUG == 0
    digitalWrite(motorEnablePin, HIGH);  // Disable motors
#endif
  }
  else
  {
#if DEBUG == 0
    digitalWrite(motorEnablePin, LOW);  // Enable motors
#endif
  }
}

int readBattery()
{
#define R1 2205.0
#define R2 2198.0 // Resistor connected with GND

  int battReading = analogRead(batteryPin) * ((R1 + R2) / R2);

  return battReading;

}



void getControlPak()
{
  unsigned long _entryTime;

  DEBUGPRINTLN3("getControlPak");

  unsigned long started_waiting_at = millis();
  bool timeout = false;
  if ( radio.available() )
  {
    DEBUGPRINTLN2("radioRead");
    // Grab the response, compare, and send to debugging spew
    radio.read( &controlPak, sizeof(controlPak) );
    int _checksum = (controlPak.throttle + controlPak.steering + controlPak.Kd + controlPak.Kp + controlPak.Kp_Throttle + controlPak.Ki_Throttle) % 256;
    if (_checksum = controlPak.checksum)
    {

      float _throttleInput = (float)controlPak.throttle / 1024;
      float _steeringInput = (float)controlPak.steering / 1024;

      if (controlPak.button1) _throttleInput =  (float)100 / 1024;
      if (controlPak.button2) _throttleInput = (float) - 100 / 1024;

      float steeringOld = _steeringInput;
      userSteering = _steeringInput;
      userThrottle = _throttleInput * MAX_THROTTLE;



      if (controlPak.button3  && !oldButton3 && operationMode == MANUAL)
      { operationMode = AUTONOMOUS;
      }
      else if (controlPak.button3  && !oldButton3 && operationMode == AUTONOMOUS) {
        operationMode = MANUAL;
      }
      oldButton3 = controlPak.button3;

      Kd_user = KD * 2 * (float)controlPak.Kd / 1024;
      Kp_user = KP * 2 * (float)controlPak.Kp / 1024;

      //			float Kp_thr_user=KP*2*(float)controlPak.Kp_Throttle/1024;
      //			float Ki_thr_user=KP*2*(float)controlPak.KI_Throttle/1024;
      //	Kp_thr_user = KP_THROTTLE*2*controlPak.Kp_thr;
      //	Ki_thr_user = (KI_THROTTLE+0.1)*2*controlPak.Ki_thr;

      DEBUGPRINT2("ControlPak ");
      DEBUGPRINT2(controlPak.steering);
      DEBUGPRINT2(" ");
      DEBUGPRINT2(controlPak.throttle);
      DEBUGPRINT2(" ");
      DEBUGPRINT2(controlPak.Kp);
      DEBUGPRINT2(" ");
      DEBUGPRINT2(controlPak.Kd);
      DEBUGPRINT2(" ");
      DEBUGPRINTLN2(controlPak.autonom);
    }

    sendDisplay();
  }
  else
  {
    if ((millis() - _entryTime) > 300)
    {
      DEBUGPRINTLN1("No data from remote");
      _entryTime = millis();
    }
  }
}

void sendDisplay()
{

  DEBUGPRINTLN2("sendDisplay");
  displayPak.steering = steering;
  displayPak.throttle = throttle;
  displayPak.angle = angle_adjusted;


  if (abs(angle_adjusted) > maxAngle) maxAngle = angle_adjusted;

  if (operationMode == MANUAL) {
    displayPak.target = target_angle;
  }
  else {
    displayPak.target = distance[MIDDLE];
  }
  displayPak.Kd = Kd;
  displayPak.Kp = Kp;
  displayPak.mode = operationMode;

  radio.stopListening();
  radio.write( &displayPak, sizeof(displayPak) );
  radio.startListening();
}

void remoteControl()
{
#if REMOTE == 1
  getControlPak();  // Read control data

  if (operationMode == MANUAL)  //Manual
  {
    if (userSteering > 0) {
      steering = (userSteering * userSteering + 0.5 * userSteering) * MAX_STEERING;
    }
    else {
      steering = (-userSteering * userSteering + 0.5 * userSteering) * MAX_STEERING;
    }

    throttle = userThrottle;
  }
#endif
}


void beep(int duration)
{
  digitalWrite(buzzPin, HIGH);
  delay(duration);
  digitalWrite(buzzPin, LOW);
  delay(100);
}

void movement()
{
  // We calculate the estimated robot speed
  // Speed = angular_velocity_of_stepper_motors - angular_velocity_of_robot(angle measured by IMU)
  actual_robot_speed_Old = actual_robot_speed;
  actual_robot_speed = (speed_m[1] - speed_m[0]) / 2; // Positive: forward

  int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 90.0; // 90 is an empirical extracted factor to adjust for real units

  //	angular_velocity = constrain(angular_velocity,-10,10);


  int16_t estimated_speed = actual_robot_speed_Old - angular_velocity;     // We use robot_speed(t-1) or (t-2) to compensate the delay
  estimated_speed_filtered = estimated_speed_filtered * 0.95 + (float)estimated_speed * 0.05;


  target_angle = speedPIControl(dt, estimated_speed_filtered, -throttle, Kp_thr, Ki_thr);
  target_angle = constrain(target_angle, -MAX_TARGET_ANGLE, MAX_TARGET_ANGLE); // limited output
  //target_angle = 0;

  // We integrate the output (acceleration)
  control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
  control_output = constrain(control_output, -500, 500); // Limit max output from control


  // The steering part of the control is injected directly on the output
  speedActual[0] = control_output + steering;
  speedActual[1] = -control_output + steering;   // Motor 2 is inverted

  // Limit max speed
  speedActual[0] = constrain(speedActual[0], -MOTORCONSTRAIN, MOTORCONSTRAIN);
  speedActual[1] = constrain(speedActual[1], -MOTORCONSTRAIN, MOTORCONSTRAIN);
}

void shutdown()
{
#if SHUTDOWN_WHEN_BATTERY_OFF == 1
  DEBUGPRINTLN1("LOW BAT!! SHUTDOWN");
  beep(3);
  // Disable steppers
  digitalWrite(4, HIGH);  // Disable motors
  while (1 == 1) {}
#endif
}

void mediumLoop()
{
  operationMode = MANUAL;
  // Medium loop 10Hz

  if (loop_counter >= 10)
  {
    loop_counter = 0;
    // if we are in autonomous mode we read the distance sensor
    if (operationMode == AUTONOMOUS)
    {
      autonomousMode();
    }
  } // Medium loop

}

void slowLoop()
{
  if (slow_loop_counter >= 99) // 2Hz
  {
    slow_loop_counter = 0;
    // Read battery status
    displayPak.battery = readBattery();

    /*DEBUGPRINT2("B");
    DEBUGPRINTLN2(displayPak.battery);
    DEBUGPRINT2(" ");
    DEBUGPRINT2(distance_sensor);
    DEBUGPRINT2(" A");
    DEBUGPRINTLN2(autonomous_mode_status);
    */

    if (displayPak.battery < BATTERY_SHUTDOWN)
    {
      shutdown();
    }
    else if (displayPak.battery < BATTERY_WARNING)
    {
      // Battery warning
      // What to do here??
      DEBUGPRINTLN1("LOW BAT!!");
      // WITA.Servo(3,servoNull+300);  // Move arm?
    }


  }  // Slow loop
}


float readAngle()
{
  float _angle = 9999.0;

  mpu.resetFIFO();
  delay(5);


  fifoCount = mpu.getFIFOCount();
  if (fifoCount >= 18)
  {

    if (fifoCount > 18) // If we have more than one packet we take the easy path: discard the buffer
    {
      digitalWrite(buzzPin, HIGH);
      DEBUGPRINTLN3("FIFO RESET!!");
      mpu.resetFIFO();
      digitalWrite(buzzPin, LOW);
    }

    else
    {
      _angle = dmpGetPhi();
      mpu.resetFIFO();  // We always reset FIFO
    }
  }
  return _angle;
}


void getAngle()
{
  float _hi = readAngle();

  while (_hi > 999.0)	{
    _hi = readAngle();
  }
  timer_value = millis();
  dt = (timer_value - timer_old);
  if (dt > 100) dt = 10;
  timer_old = timer_value;

  angle_adjusted_Old = angle_adjusted;
  angle_adjusted = _hi;
  displayPak.angle = angle_adjusted;

}

void initPins()
{
  // SET PINS
  pinMode(motorEnablePin, OUTPUT); // ENABLE MOTORS
  pinMode(motor1StepPin, OUTPUT); // STEP MOTOR 1 PORTD,7
  pinMode(motor1DirPin, OUTPUT); // DIR MOTOR 1
  pinMode(motor2StepPin, OUTPUT); // STEP MOTOR 2 PORTD,6
  pinMode(motor2DirPin, OUTPUT); // DIR MOTOR 2
  digitalWrite(motorEnablePin, HIGH);  // Disbale motors

  pinMode(buzzPin, OUTPUT);
}



void initMCU()
{
  // initialize MPU6050
  DEBUGPRINTLN0("MPU6050 initialization start...");
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);  //10,20,42,98,188
  mpu.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
  mpu.setSleepEnabled(false);

  mpu.resetFIFO();
  DEBUGPRINTLN0("MPU6050 done...");

  DEBUGPRINTLN0(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    DEBUGPRINTLN0("DMP done, waiting 15 sec ...");

  } else
  { // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    DEBUGPRINT0(F("DMP Initialization failed (code "));
    DEBUGPRINT0(devStatus);
    DEBUGPRINTLN0(F(")"));
  }

  // Gyro calibration
  // The robot must be steady during initialization
#if DEBUG==0
  for (int _ww = DMP_WAIT; _ww > 0; _ww--)
  {
    delay(800);   // Time to settle things... the bias_from_no_motion algorithm needs some time to take effect and reset gyro bias.
    beep(200);
  }
  beep(200);

  //	digitalWrite(ledPin,LOW);
#endif

  //Adjust sensor fusion gain
  DEBUGPRINTLN0("Adjusting DMP sensor fusion gain...");
  dmpSetSensorFusionAccelGain(0x20);

  // verify connection
  DEBUGPRINTLN0("Testing device connections...");
  DEBUGPRINTLN0(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  if (!mpu.testConnection()) beep(1000);
}

void initSteppers()
{
  //We are going to overwrite the Timer1 to use the stepper motors

  // STEPPER MOTORS INITIALIZATION
  // TIMER1 CTC MODE
  DEBUGPRINTLN0("Initializing Stepper Motors ...");
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |=  (1 << WGM12);
  TCCR1A &= ~(1 << WGM11);
  TCCR1A &= ~(1 << WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3 << COM1A0);
  TCCR1A &= ~(3 << COM1B0);

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);

  //OCR1A = 125;  // 16Khz
  //OCR1A = 100;  // 20Khz
  OCR1A = 80;   // 25Khz
  TCNT1 = 0;

  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
#if DEBUG ==0
  digitalWrite(motorEnablePin, LOW);   // Enable stepper drivers
#else
  digitalWrite(motorEnablePin, HIGH); //disable motors
#endif
  DEBUGPRINTLN0("Stepper Motors done...");
}

void NRFInit()

// Set up nRF24L01 radio on SPI bus plus pins CE & CSN
{

  DEBUGPRINTLN0("NRF initialization start ...");
  radio.begin();

  // This simple sketch opens two pipes for these two nodes to communicate
  // back and forth.
  // Open 'our' pipe for writing
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1, pipes[0]);

  int payloadLenth = max(sizeof(displayPak), sizeof(controlPak));
  DEBUGPRINT0("Max Payload "); DEBUGPRINTLN0(payloadLenth);

  if (payloadLenth > 32) beep(1000);
  radio.setChannel(CHANNEL);
  radio.setDataRate( DATARATE ) ;
  radio.setPALevel( RF24_PA_MAX ) ;

  radio.enableDynamicPayloads() ;
  radio.setAutoAck( true ) ;
  radio.powerUp() ;
  radio.startListening();

  //
  // Dump the configuration of the rf unit for debugging
  //

# if DEBUG>0
  radio.printDetails();
#endif

  if (!radio.isPVariant()) beep(1000);
  DEBUGPRINTLN0("NRF initialization done ...");
}


void readDistanceSensor()
{
  unsigned long start = millis();


  while ((readTiny(7) < 255) && (millis() - start) < 1000) {  // wait for first byte
  }

  for (place = 0; place < 3; place++) {

    distance[place] = readTiny(7);
  }

  if (distance[LEFT] < distance[MIDDLE] && distance[LEFT] < distance[RIGHT]) {

    dist_min = distance[LEFT];
    dir_min = LEFT;
  }
  else {
    if (distance[MIDDLE] < distance[RIGHT]) {

      dist_min = distance[MIDDLE];
      dir_min = MIDDLE;
    }
    else {
      dist_min = distance[RIGHT];
      dir_min = RIGHT;
    }
  }
  if (distance[MIDDLE] > 50) {
    dist_min = distance[MIDDLE];
    dir_min = MIDDLE;
  }
}


void autonomousMode()  // ???
{
  readDistanceSensor();
  displayPak.number = autStat;
  int statOld = autStat;


  switch (autStat)
  {
    case WALK:   //walk
      {
        displayPak.text_ref = 0;

        throttle = map(distance[MIDDLE], distCorr, 100, throttleAutoMax / 2, throttleAutoMax);
        steering = 0;
        if ((distance[MIDDLE] <= distCorr)  || (dist_min <= distCorr / 2)) {
          autStat = STEER;
        }
        break;
      }

    case STEER:   // avoid
      {
        displayPak.text_ref = 1;
        throttle = map(distance[MIDDLE], distCorr, 0, throttleAutoMax / 2, 0);

        if ((millis() - entrySteer) > 1) { // since last steering some milliseconds for reaction

          switch (dir_min) {
            case LEFT:
              entrySteer = millis();
              steering = aSteering(RIGHT);  // go to right
              break;

            case RIGHT:
              entrySteer = millis();
              steering = aSteering(LEFT);  // go to left
              break;

            case MIDDLE:
              entrySteer = millis();
              if (distance[LEFT] < distance[RIGHT]) {
                steering = aSteering(RIGHT);
              } // go to right}
              else {
                steering = aSteering(LEFT);
              } // go to left}
              break;
          }

          if (dist_min > distCorr) {
            firsttimeSteer = true;
            autStat = WALK;
          }
          if ((distance[MIDDLE] < distCloseup) || (dist_min < distCloseup / 2)) {
            firsttimeSteer = true;
            autStat = TURN;
          }
          break;
        }
      }

    case TURN:   // turn on place
      {
        displayPak.text_ref = 2;
        if (firsttimeTurn) {
          entryTime = millis();
          firsttimeTurn = false;
          // turn in the "best" direction
          throttle = 0;
          if (distance[LEFT] < distance[RIGHT]) {
            steering = aSteering(RIGHT); // go to right
          }
          else {
            steering = aSteering(LEFT);  // goto left
          }
        }

        if (distance[MIDDLE] > distCloseup)
        {
          firsttimeTurn = true;
          autStat = STEER;
        }
        if (millis() - entryTime > 3000) {
          firsttimeTurn = true;
          autStat = BLOCKED;
        }

        break;
      }

    case BLOCKED:   //stop
      {
        displayPak.text_ref = 9;
        throttle = 0;
        steering = 0;
        DEBUGPRINT1("Case BLOCKED Distance: ");
        DEBUGPRINTLN1(distance[MIDDLE]);
        if (distance[MIDDLE] > distCloseup) autStat = STEER;
        break;
      }

otherwise:
      {
        autStat = BLOCKED;
        beep(1000);
      }
  }

}



//===============================================================================================

void setup()
{

  Serial.begin(115200);
  initPins();
  //	pinMode(ledPin,OUTPUT);



  // Join I2C bus
  DEBUGPRINTLN0("I2C initialization start");
  Wire.begin();
  // 4000Khz fast mode
  TWSR = 0;
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2;
  TWCR = 1 << TWEN;
  DEBUGPRINTLN0("I2C done...");
  initMCU();


  // Init NRF24L01
#if REMOTE == 1
  NRFInit();
#endif
  initSteppers();


  DEBUGPRINT0("Free RAM: ");
  DEBUGPRINTLN0(freeRam());
  DEBUGPRINT0("MAX_THROTTLE: ");
  DEBUGPRINTLN0(MAX_THROTTLE);
  DEBUGPRINT0("MAX_STEERING: ");
  DEBUGPRINTLN0(MAX_STEERING);
  DEBUGPRINT0("MAX_TARGET_ANGLE: ");
  DEBUGPRINTLN0(MAX_TARGET_ANGLE);
  // Little motor vibration to indicate that robot is ready
  for (uint8_t k = 0; k < 3; k++)
  {
    setMotorSpeed(0, 3);
    setMotorSpeed(1, -3);
    delay(150);
    setMotorSpeed(0, -3);
    setMotorSpeed(1, 3);
    delay(150);
  }
  //	initLEDS();
  timer_old = millis();
  autStat = WALK;

  speedRequired[0] = 0;
  speedRequired[1] = 0;



  DEBUGPRINTLN0("Setup done");
}


//===============================================================================================
//===============================================================================================

// Main loop
unsigned long loopentry;
void loop()
{
  Serial.println(millis() - loopentry);
  loopentry = millis();


  remoteControl();
  getAngle();
  movement();


  // default motorSpeed
  speedRequired[0] = speedActual[0];
  speedRequired[1] = speedActual[1];
  MAX_ACCEL = 7;


  switch (robotStat)
  {
    case up:
      {
        Kp = Kp_user;   // CONTROL GAINS FOR NORMAL OPERATION
        Kd = Kd_user;
        Kp_thr = KP_THROTTLE;
        Ki_thr = KI_THROTTLE;
        speedup = 10;

        // exit
        if (abs(angle_adjusted) > NEARUP && abs(angle_adjusted) < DOWN) robotStat = gettingUp;
        break;
      }


    case gettingUp:
      {
        getAngle();
        // more agressive PID factors

        // exit
        if (abs(angle_adjusted) > DOWN) robotStat = down;
        if (abs(angle_adjusted) < NEARUP) robotStat = up;
        break;
      }

    case manualGetUp:  // Manual Getup
      {
        delay(2000);
        beep(100);
        throttle = 0;
        steering = 0;
        angle_adjusted_Old = 0;
        estimated_speed_filtered = 0;
        timer_old = millis();
        movement();

        // exit
        robotStat = up;
        break;
      }

    case down:					//down
      {
        userSteering = 0;
        throttle = 0;
        speedup = 0;

        // exit
        if (abs(angle_adjusted) <= DOWN) robotStat = manualGetUp;
        break;
      }


    default:
      {
        beep(1000);
        break;
      }

  }
  setMotorSpeed(0, speedRequired[0] * speedup / 10);
  setMotorSpeed(1, speedRequired[1] * speedup / 10);
  loop_counter++;

  slow_loop_counter++;
  mediumLoop();
  slowLoop();
}


byte readTiny(int address) {
  unsigned long start = millis();
  Wire.requestFrom(address, 1);                  // The TinyWire library only allows for one byte to be requested at a time
  while ((Wire.available() == 0) && (millis() - start) < 1000)  ;
  byte hh = Wire.read();
  return hh;
}

int aSteering(boolean direction) {

  int hi = map(dist_min, 0, distCorr, 60, 0);

  int room = abs(distance[LEFT] - distance[RIGHT]);
  //  hi = map(0, 100, 0, 60);

  //		hi= 50;

  if (direction == LEFT) {
    hi = -hi;
  }

  return hi;
}
