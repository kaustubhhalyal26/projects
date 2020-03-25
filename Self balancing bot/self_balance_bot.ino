#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <FlexiTimer2.h>

MPU6050 mpu;

//***************read_angle with comp filter*********************//
int16_t ax, ay, az, gx, gy, gz;
int16_t rawAccX, rawAccY, rawAccZ, rawTemp,rawGyroX, rawGyroY, rawGyroZ;
volatile double gyroXoffset=4, gyroYoffset=6, gyroZoffset=18;
volatile double temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
float angleGyroX, angleGyroY, angleGyroZ,angleAccX, angleAccY, angleAccZ;
float angleX, angleY, angleZ;
float interval;
long preInterval;
float accCoef=0.02, gyroCoef=0.98;
//***************read_angle with comp filter*********************//

//***************Motors*************//
#define MA1   13
#define MA2   9
#define EA    10
#define MB1   7
#define MB2   4
#define EB    6
#define ENCA1 19
#define ENCA2 18
#define ENCB1 2
#define ENCB2 3

// Define Operating Modes
#define COAST 0
#define FORWARD 1
#define BACK  2
#define LEFT  3
#define RIGHT 4
#define BRAKE 5

// Define Minimum PWM Values for Motors
//#define LEFT_PWM_MIN 35
//#define RIGHT_PWM_MIN 42

volatile float left_encoder_count=0;
volatile float right_encoder_count=0;

//***************Motors*************//



#define MagF 51
#define GREEN_pin 47
#define RED_pin 43 
#define Common_pin 45
#define buzz_pin 31

int motorSpeedA = 0;
int motorSpeedB = 0;




// Angle Values
#define TILT_ANGLE_OFFSET -4    // Angle offset to correct CG of robot
#define COMP_FILTER_ALPHA 0.98
#define SLOPE_ANGLE 2.8

// Speed values
#define FULL_SPEED 100        // pos increment rate
#define FORWARD_SPEED 185
#define REVERSE_SPEED 170
#define SLOPE_SPEED 155
#define TURN_SPEED 60

// Minimum PWM Values for Motors
#define LEFT_PWM_MIN 50
#define RIGHT_PWM_MIN 10

// Rotation Proportional Gains
#define LEFT_GAIN 5
#define RIGHT_GAIN 5

// External Variables
volatile unsigned long int time_ms = 0;
volatile unsigned long int time_sec = 0;

// Global Variables
float slope_offset=0, move_offset=0, max_angle_vel=4, max_angle_enc=2;
volatile float accel_angle=0, gyro_angle=0;
volatile float rotation_left=0, rotation_right=0;
volatile float left_RPM=0, right_RPM=0, left_prev_count=0, right_prev_count=0;
unsigned long last_task_time_PID=0;

// Flags
bool STOP_FLAG = true;
bool ROTATION_FLAG = false;
bool SLOPE_FLAG = false;



struct PID
{
  volatile float con_KP;      // Conservative proportional gain
  volatile float con_KI;      // Conservative integral gain
  volatile float con_KD;      // Conservative derivative gain
  
  volatile float agr_KP;      // Aggressive proportional gain
  volatile float agr_KI;      // Aggressive integral gain
  volatile float agr_KD;      // Aggressive derivative gain
  
  volatile float set_point;   // Set point value
  volatile float error;     // Error value
  volatile float pos;    // Current pos
  volatile float last_pos; // Previous pos
  
  volatile float integral;    // Integral sum
  volatile float derivative;    // Derivative term
  volatile float output;      // PID output 
  volatile int direc;     // Controller direc
};

// Structure Initializations
PID angle    = {20, 0, 25, 20, 0, 40};
PID velocity = {19, 0, 10, 10, 0, 0, 0};
PID encoder  = {10, 0, 0, 10, 0, 0, 0};


//Controller
struct JoystickController
{
  int x_position;   // Joystick X-Axis ADC Value
  int y_position;   // Joystick Y-Axis ADC Value
  
  bool button_1;    // Push Button 1 Value
  bool button_2;    // Push Button 2 Value
  bool button_3;    // Push Button 3 Value
  bool button_4;    // Push Button 4 Value
  
  // De-bounce time for the push buttons
  unsigned long b1_time;
  unsigned long b2_time;
  unsigned long b3_time;
  unsigned long b4_time;
};


JoystickController joystick = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};





void setup()
{
    Wire.begin();
    mpu.initialize(); 
    Serial.begin(9600);
    Serial2.begin(9600);

    motors_init();

    // Set PID controller direcs
    angle.direc = 1;
    velocity.direc = -1;
    encoder.direc = -1;
    
    FlexiTimer2::set(20, interrupt);    //5ms
    FlexiTimer2::start();                 
}

void loop()
{
//  Serial.print(right_encoder_count);
//  Serial.print("\t");
//  Serial.print(left_encoder_count);
//  Serial.print("\n");
  Serial.println(angle.error);

}




void motor_pin_config()
{
  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);
  pinMode(EA, OUTPUT);
  pinMode(EB, OUTPUT);  
}


void encoder_pin_config()
{
  // Set as input pins and enable internal pull-up
  pinMode(ENCA1, INPUT); // Encoder 1 - Channel A 
  pinMode(ENCA2, INPUT); // Encoder 1 - Channel B 
  
  pinMode(ENCB1, INPUT); // Encoder 2 - Channel A 
  pinMode(ENCB2, INPUT); // Encoder 2 - Channel B
  
  // Attach interrupts for the encoder input pins
  attachInterrupt(digitalPinToInterrupt(ENCA1), left_encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB1), right_encoder_interrupt, CHANGE);
}

void left_encoder_interrupt()
{
  int state = digitalRead(ENCA1);
  if(digitalRead(ENCA2)) 
  state ? left_encoder_count-- : left_encoder_count++;
  else 
  state ? left_encoder_count++ : left_encoder_count--;
}


void right_encoder_interrupt()
{
  int state = digitalRead(ENCB1);
  if(digitalRead(ENCB2)) 
  state ? right_encoder_count++ : right_encoder_count--;
  else 
  state ? right_encoder_count-- : right_encoder_count++;
}



void motors_init()
{
  motor_pin_config();
  encoder_pin_config();
}





void interrupt()
{
  sei();                                           
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC Get MPU6050 six-axis data ax ay az gx gy gz
  read_angle(ax,ay,az,gx,gy,gz);
  RPM_motor();
  read_joystick();  // Read the raw data from Joystick Controller
//  led_scheduler();  // Run the LED scheduler for status/beacon indicator
//  buzz_scheduler(); // Run the buzzer scheduler and play non-blocking RTTTL tones
//  
  // PID task scheduling every 20ms
 // if ((epoch() - last_task_time_PID) >= 5)
  //{
  //  last_task_time_PID = epoch();
    
    steer_robot();  // Update the set-points for the various PID loop
    compute_PID();  // Compute PID values
    
    // Update the motor speed and direction
    update_motors(angle.output, rotation_left, rotation_right);
//  }
}



void RPM_motor()
{
  // Make a local copy of the global encoder count
  volatile float left_current_count = left_encoder_count;
  volatile float right_current_count = right_encoder_count;
  
  //     (Change in encoder count) * (60 sec/1 min)
  // RPM = __________________________________________
  //     (Change in time --> 20ms) * (PPR --> 840)
  left_RPM = (float)(((left_current_count - left_prev_count) * 60)/(0.005*135));
  right_RPM = (float)(((right_current_count - right_prev_count) * 60)/(0.005*135));
  
  // Store current encoder count for next iteration
  left_prev_count = left_current_count;
  right_prev_count = right_current_count;
}

//*************Motors****************//


float encoder_count()
{
  return (left_encoder_count + right_encoder_count);
}



void set_motor_PWM(int motor, unsigned char motor_speed)
{
  motor_speed=abs(motor_speed);
  if (motor==LEFT)  
    analogWrite(EA, motor_speed);
  if (motor==RIGHT) 
    analogWrite(EB, motor_speed);
}


void set_motor_pin(int motor, bool pin1, bool pin2)
{
  if (motor==LEFT){
    digitalWrite(MA1,pin1);
    digitalWrite(MA2,pin2);
    }
  if (motor==RIGHT){
    digitalWrite(MB1,pin1);
    digitalWrite(MB2,pin2);
    }
}


void set_motor_mode(int motor, int mode)
{
  if (mode==FORWARD)  set_motor_pin(motor, LOW, HIGH);
  if (mode==BACK)     set_motor_pin(motor, HIGH, LOW);
  if (mode==COAST)    set_motor_pin(motor, LOW, LOW);
  if (mode==BRAKE)    set_motor_pin(motor, HIGH, HIGH);
}

void drive_motor(int motor, float PWM_value, float min_value)
{
  // Coast the motors
  if (PWM_value == 0)
  {
    set_motor_PWM(motor, 0);
    set_motor_mode(motor, COAST);
  }
  
  // Move the robot forward
  else if (PWM_value > 0)
  {
    PWM_value = map(PWM_value, 0, 255, min_value, 255);   // Map the PWM values
    set_motor_PWM(motor, (unsigned char)PWM_value);     // Set motor speed
    set_motor_mode(motor, FORWARD);             // Set motor direc
  }
  
  // Move the robot back
  else if (PWM_value < 0)
  {
    PWM_value = map(PWM_value, 0, -255, min_value, 255);  // Map the PWM values
    set_motor_PWM(motor, (unsigned char)PWM_value);     // Set motor speed
    set_motor_mode(motor, BACK);              // Set motor direc
  }
}


void update_motors(float PID_output, float left_offset, float right_offset)
{
  float left_PWM=0, right_PWM=0;
  
  // Add rotation offsets and constrain the output
  left_PWM = constrain(PID_output + left_offset, -255, 255);
  right_PWM = constrain(PID_output + right_offset, -255, 255);
  
  // Drive the motors
  drive_motor(LEFT, left_PWM, LEFT_PWM_MIN);
  drive_motor(RIGHT, right_PWM, RIGHT_PWM_MIN);
}

//*************Motors****************//







unsigned long epoch()
{
  unsigned long elapsed_time;
  elapsed_time = time_sec*1000 + time_ms;
  return elapsed_time;
}


void read_angle(int16_t rawAccX,int16_t rawAccY,int16_t rawAccZ,int16_t rawGyroX,int16_t rawGyroY,int16_t rawGyroZ)
{
  accX = ((int16_t)rawAccX) / 16384.0;
  accY = ((int16_t)rawAccY) / 16384.0;
  accZ = ((int16_t)rawAccZ) / 16384.0;

  angleAccX = atan2(accY, sqrt(accZ * accZ + accX * accX)) * 360 / 2.0 / PI;
  angleAccY = atan2(accX, sqrt(accZ * accZ + accY * accY)) * 360 / -2.0 / PI;

  gyroX = ((int16_t)rawGyroX) / 65.5;
  gyroY = ((int16_t)rawGyroY) / 65.5;
  gyroZ = ((int16_t)rawGyroZ) / 65.5;

  gyroX -= gyroXoffset;
  gyroY -= gyroYoffset;
  gyroZ -= gyroZoffset;

  interval = (millis() - preInterval) * 0.001;

  angleGyroX += gyroX * interval;
  angleGyroY += gyroY * interval;
  angleGyroZ += gyroZ * interval;

  angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
  angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
  angleZ = angleGyroZ;

  angle.pos=angleX;

  preInterval = millis();
}



//***********Controller************//

void handle_buttons()
{
  // Button 4 - Move Forward
  if ((joystick.button_1 == HIGH) && ((epoch() - joystick.b1_time) >= 1))
  {
    joystick.b1_time = epoch();
    
    // Update set-point
    encoder.set_point += 0.1*FULL_SPEED; // Increment the Position Set Point at a Defined Speed
    velocity.set_point = FORWARD_SPEED;
    move_offset = -0.05;
    
    // Update flags
//    set_led_indicators(false, true, false, false, false, false);
    
    // Set STOP_FLAG to False to avoid Static Balance/Position Holding
    STOP_FLAG = false;
    SLOPE_FLAG = false;
  }
  
  // Button 1 - Move Back
  else if ((joystick.button_2 == HIGH) && ((epoch() - joystick.b2_time) >= 1))
  {
    joystick.b2_time = epoch();
    
    // Update set-point
    encoder.set_point -= 0.1*FULL_SPEED; // Increment the Position Set Point at a Defined Speed
    velocity.set_point = -REVERSE_SPEED;
    move_offset = 0.05;
    
    // Update flags
//    set_led_indicators(false, false, true, false, false, false);
    
    // Set STOP_FLAG to False to avoid Static Balance/Position Holding
    STOP_FLAG = false;
    SLOPE_FLAG = false;
  }
  
  // Button 2 - Slope Mode
  else if ((joystick.button_3 == HIGH) && ((epoch() - joystick.b3_time) >= 1))
  {
    joystick.b3_time = epoch();
    
    // Gradually increase slope offset angle
    slope_offset -= 0.1;
    slope_offset = constrain(slope_offset, -SLOPE_ANGLE, 0);
    velocity.set_point = SLOPE_SPEED;
    
    // Update flags
    //set_led_indicators(false, false, false, false, false, true);
    
    // Set STOP_FLAG to False to avoid Static Balance/Position Holding
    STOP_FLAG = false;
    SLOPE_FLAG = true;
  }
  
  // Button 0 - Hold Position
  else if (!STOP_FLAG)
  {
    // Reset count to current position
    left_encoder_count = right_encoder_count = 0;
    encoder.set_point = encoder_count();
    
    velocity.set_point = 0;
    slope_offset = 0;
    move_offset = 0;
    
    // Update flags
//    set_led_indicators(true, false, false, false, false, false);
    STOP_FLAG = true;
    SLOPE_FLAG = false;
  }

  // Button 3 - Sound Buzzer
//  if ((joystick.button_4 == HIGH) && ((epoch() - joystick.b4_time) >= 50))
//  {
//    joystick.b4_time = epoch();
//    if (!read_buzzer_state())
//    {
//      set_buzzer_state(true);
//      set_buzz_time();
//    }
//  }
}



void steer_robot()
{
  // Update the rotation variables
  switch(joystick.x_position)
  {
    // Zone -2/-1 --> Rotate Right
    case -2:
    case -1:
      // Set the turning speed
      rotation_left = TURN_SPEED;
      rotation_right = -TURN_SPEED;     
//      set_led_indicators(false, false, false, false, true, false);
      ROTATION_FLAG = true;
    break;
    
    // Neutral Position
    case 0:
      rotation_left = 0;
      rotation_right = 0;     
//      set_led_indicators(true, false, false, false, false, false);
      ROTATION_FLAG = false;
    break;
    
    // Zone 1/2 --> Rotate Left
    case 1:
    case 2:
      // Set the turning speed
      rotation_left = -TURN_SPEED;
      rotation_right = TURN_SPEED;      
//      set_led_indicators(false, false, false, true, false, false);
      ROTATION_FLAG = true;
    break;
  }
  
  // Handle the Controller's Push Button Inputs
  handle_buttons();
}
//***********Controller************//



//*********PID**********//
//
void compute_rotation_PID()
{
  // Avoid rotational correction when joystick controller is being used to turn the robot
  if (ROTATION_FLAG)
  {
    left_encoder_count = right_encoder_count; // Equalize the offset  
    return;
  }
  
  // Correct rotation drift by computing the difference between the encoder poss
  rotation_left = -1 * LEFT_GAIN * (left_encoder_count - right_encoder_count) * 0.05;
  rotation_right = RIGHT_GAIN * (left_encoder_count - right_encoder_count) * 0.05;
}



void compute_velocity_PID()
{
  float velocity_KP = 0;
  
  // Get current linear velocity measured using encoders
  velocity.pos = (left_RPM + right_RPM)/2.0; // Average of both motor RPMs
  
  // Compute error
  velocity.error = velocity.set_point - velocity.pos;
  
  // Normal motion
  if (!STOP_FLAG)
  {
    velocity_KP = velocity.con_KP;
    if (abs(velocity.error) < 60) max_angle_vel = 2.5;
    if (abs(velocity.error) >= 60) max_angle_vel = 4;
  }
  
  // Slope
  else if (SLOPE_FLAG)
  {
    velocity_KP = velocity.agr_KP;
    max_angle_vel = 6;
  }
  
  // Static balance
  else
  {
    velocity_KP = velocity.con_KP;
    max_angle_vel = 2;
  }
  
  // Compute derivative
  velocity.derivative = velocity.pos - velocity.last_pos;

  // Compute integral and prevent wind-up 
  velocity.integral += velocity.con_KI*0.001*velocity.error;
  velocity.integral = constrain(velocity.integral, -max_angle_vel, max_angle_vel);

  // Compute velocity PID output
  velocity.output = (velocity_KP*0.001*velocity.error) + velocity.integral - (velocity.con_KD*0.001*velocity.derivative);
  
  // Constrain velocity PID output to PWM range
  velocity.output = velocity.direc * constrain(velocity.output, -max_angle_vel, max_angle_vel);
  
  // Store current velocity for next iteration
  velocity.last_pos = velocity.pos;
}

void compute_encoder_PID()
{
  float encoder_KP = 0;
  
  // Get current encoder pos
  encoder.pos = encoder_count();
  
  // Compute pos error
  encoder.error = encoder.set_point - encoder.pos;
  
  // Normal motion
  if (!STOP_FLAG)
  {
    encoder_KP = encoder.con_KP;
    max_angle_enc = 2;
  }
  
  // Slope
  else if (SLOPE_FLAG)
  {
    encoder_KP = encoder.con_KP;
    max_angle_enc = 4;
  }
  
  // Static balance
  else
  {
    encoder_KP = encoder.agr_KP;
    max_angle_enc = 2;
  }

  // Compute encoder PID output
  encoder.output = (encoder_KP*encoder.error*0.0001);
  
  // Constrain the encoder PID output
  encoder.output = encoder.direc * constrain(encoder.output, -max_angle_enc, max_angle_enc);
  
  //if (abs(encoder.error) < 50) encoder.output = 0;  // For smoothing output
}


void compute_angle_PID()
{
  float angle_KP=0, angle_KI=0, angle_KD=0;
  float angle_pos;
  
  // Obtain the current tilt angle and make a copy to avoid
  // abrupt changes during current PID computation loop
  angle_pos = angle.pos;
  
  // Add all the offsets to the angle set-point
  angle.set_point = TILT_ANGLE_OFFSET + move_offset + slope_offset + encoder.output + velocity.output;
  
  // Compute tilt angle error
  angle.error = angle.set_point - angle_pos;
  
  // Turn motors off if robot falls beyond recoverable angle and await human rescue
  if (abs(angle.error) >= 45)
  {
    angle.output = 0;
    return;
  }

  // Conservative PID gains for |errors| < 3 degress
  if (abs(angle.error) < 5.0)
  {
    angle_KP = angle.con_KP;
    angle_KI = angle.con_KI;
    angle_KD = angle.con_KD;
  }
  
  // Aggressive PID gains for |errors| >= 3 degress
  else
  {
    angle_KP = angle.agr_KP;
    angle_KI = angle.agr_KI;
    angle_KD = angle.agr_KD;
  }
  
  // Compute derivative term
  angle.derivative = angle_pos - angle.last_pos;
  
  // Compute integral sum
  angle.integral += angle_KI*angle.error;
  
  // Constrain integral term to prevent wind-up
  angle.integral = constrain(angle.integral, -255, 255);
  
  // Compute angle PID output
  angle.output = (angle_KP*angle.error) + (angle.integral) - (angle_KD*angle.derivative);
  
  // Constrain angle PID output to PWM range
  angle.output = constrain(angle.output, -255, 255);
  
  //if (abs(angle.error) < 0.2) angle.output = 0;   // For smoothing output
  
  // Store variable for next iteration
  angle.last_pos = angle_pos;
}


void compute_PID()
{
  compute_encoder_PID();
  compute_velocity_PID();
  compute_angle_PID();
  //  compute_rotation_PID();
}

//*********PID**********//

//********Controller**********//
int get_joystick_zone(unsigned char low_byte, unsigned char high_byte, int offset=0)
{
  int output=0, zone=0;
  offset=0;  
  // Combine bytes and constrain input
  output = constrain((high_byte*255 + low_byte - offset), 0, 1023);
  
  // Map values to -100 and 100 range
  output = map(output, 0, 1023, -100, 100);
  
  // Assign different zone based on the magnitude of the joystick position
  // ±2 -> Extreme ends, 0 -> Neutral position
  if (abs(output)<=20) zone = 0;
  else if ((abs(output)>20) && (abs(output)<=60)) zone = 1;
  else if ((abs(output)>60) && (abs(output)<=100)) zone = 2;
  
  if (output<0) zone *= -1;
  return zone;
}


void read_joystick()
{
  int sum = 131;
  unsigned char byte_discard, checksum;
  unsigned char digital_data = 0;
  unsigned char AD0[2]={0, 0}, AD1[2]={0, 0};

  //126,0,14,131,0,4,48,0,1,6,28,0,0,3,255,3,255,33,
  //7E,0,E,83,0,4,2A,0,1,6,1C,0,0,3,FF,3,FF,27,

  // Check frame length
  if (Serial2.available()>=21)
  {
    // Check Start Byte of the API Frame
    if (Serial2.read() == 0x7E)
    {
      // Read and discard 2 bytes
      for (int i=0; i<2; i++)
      byte_discard = Serial2.read();
      
      // Check Frame Type
      if(Serial2.read() != 0x83) return;
      
      // Read and discard 8 bytes
      for (int i=0; i<8; i++)
      sum += Serial2.read();
      
      // Read Digital Byte
      digital_data = Serial2.read();
      sum += digital_data;
      
      // Read ADC Data
      AD0[1] = Serial2.read();
      AD0[0] = Serial2.read();
      AD1[1] = Serial2.read();
      AD1[0] = Serial2.read();
      sum = sum + AD0[0] + AD0[1] + AD1[0] + AD1[1];
      
      // Read Checksum
      checksum = 0xFF - (0xFF & (unsigned char)sum);
      byte_discard = Serial2.read();
      
      // Discard frame if checksum does not match
      if (byte_discard != checksum) return;
      
      // Update Controller Variables
      if ((digital_data & 0x04) == 0x04) joystick.button_1 = HIGH;
      else joystick.button_1 = LOW;
      
      if ((digital_data & 0x08) == 0x08) joystick.button_2 = HIGH;
      else joystick.button_2 = LOW;
      
      if ((digital_data & 0x10) == 0x10) joystick.button_3 = HIGH;
      else joystick.button_3 = LOW;
      
      if ((digital_data & 0x40) == 0x40) joystick.button_4 = HIGH;
      else joystick.button_4 = LOW;
      
      // Map ADC values to Zones
      joystick.x_position = get_joystick_zone(AD1[0], AD1[1], 9);
      joystick.y_position = get_joystick_zone(AD0[0], AD0[1], 20);
    }
  }
}
