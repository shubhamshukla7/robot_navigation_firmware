//Including header files
#include <PID_v1.h>

//Definition of constants -- Tuning Parameters for PID
#define KP 8.0
#define KI 32.0
#define KD 0.1

//Definition of constants -- Encoders
#define LEFT_ENCODER 20
#define RIGHT_ENCODER 21

//Definition of constants -- Bluetooth Messaging Protocol Constants
#define INTEGER_BUILDER_ARR_SIZE 8
#define ARRAY_DELIMITER 127
#define SPACE_CHARACTER ' '
#define MOTOR 0                        //Tag for Motor Commands
#define ASTERICK 42
#define BANG 33
#define COMMAND_CHAR_ARRAY 64
#define DELAY 5

//Definition of Variables -- Operating Modes
typedef enum mode{NORMAL,DEBUG};
mode operatingMode=NORMAL;

//Definition of Variables -- Encoders
volatile int leftEncoder=0;
volatile int rightEncoder=0;
int previousLeftEncoder=0;
int previousRightEncoder=0;

//Definition of Variables -- Motors
const byte leftMotor_PWM=11;
const byte leftMotor_Forward=6;
const byte rightMotor_PWM=10;
const byte rightMotor_Forward=12;

//Definition of Variables -- Analog Multiplexer
const byte mux_s0=7;
const byte mux_s1=8;
const byte mux_s2=9;
const byte mux_signal=A3;

//Definition of Variables -- PID
double leftPIDInput,leftPIDOutput,leftPIDSetPoint;      //Variables for Left Motor PID
double rightPIDInput,rightPIDOutput,rightPIDSetPoint;   //Variables for Right Motor PID
int leftSpeed,rightSpeed;

//Definition of Variables -- Bluetooth Messaging Protocol
volatile byte x,px,y,py;
volatile byte x2,px2,y2,py2;

//Definition of Variables -- Cruise Control
unsigned long msgTime=0;
unsigned long speedSampleTime=0;
unsigned long speedSampleInterval=200;                 //Time interval between 2 successive speed calculations (in milliseconds)

//Definition of Objects -- PID
PID leftPID(&leftPIDInput,&leftPIDOutput,&leftPIDSetPoint,KP,KI,KD,DIRECT);
PID rightPID(&rightPIDInput,&rightPIDOutput,&rightPIDSetPoint,KP,KI,KD,DIRECT);

//Prototype Function Declarations
void build_command(int command[],char received_char_arr[]);    //Builds a command from a character array received via the designed Bluetooth messaging protocol
void send_command(int command[]);                              //Sends the built command to the robot
void setLeftspeed(int duty);                                   //Sets the duty cycle for left motor's PWM
void setRightspeed(int duty);                                  //Sets the duty cycle for right motor's PWM
void cruiseControl(int targetSpeed);                           //Performs cruise control on the Robot
void debug();                                                  //Enters debug mode and performs tests to ensure proper functioning of the robot
void leftEncoder();
void rightEncoder();
float getObstacleDistance(int pin);                            //Reads the distance of an obstacle using a GP2Y0A21 IR Sharp Sensor
int readAnalogMux(byte mux_pin);                               //Switches the Analog Multiplexer on and samples data from the attached Analog Port

//Setup function
void setup(){
  if(operatingMode==DEBUG)
    debug();
  //Setting appropriate pin configurations -- Encoders
  pinMode(LEFT_ENCODER,INPUT);
  pinMode(RIGHT_ENCODER,INPUT);
  //Setting appropriate pin configurations -- Analog Multiplexers
  pinMode(mux_s0,OUTPUT);
  pinMode(mux_s1,OUTPUT);
  pinMode(mux_s2,OUTPUT);
  pinMode(mux_signal,INPUT);
  //Setting appropriate pin configurations -- Motors
  pinMode(leftMotor_FWD,OUTPUT);
  pinMode(leftMotor_PWM,OUTPUT);
  pinMode(rightMotor_FWD,OUTPUT);
  pinMode(rightMotor_PWM,OUTPUT);
  
  //Attaching interrupts to encoders
  attachInterrupt(0,leftEncoder,RISING);
  attachInterrupt(1,rightEncoder,RISING);
  
  //Initializing a few variables
  leftPIDSetpoint=0;
  rightPIDSetpoint=0;
  leftPID.SetOutputLimits(-255,255);
  rightPID.SetOutputLimits(-255,255);
  leftPID.SetSampleTime(speedSampleInterval);
  rightPID.SetSampleTime(speedSampleInterval);
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  
  //Starting the Serial Monitor with a baud rate of 115200
  Serial.begin(115200);
}

void loop{
  char command_array[COMMAND_CHAR_ARRAY]={0};
  int index=0;
  bool inside_a_message=false;
  if(Serial.available()>0){
    while(Serial.available()>0){
      char received_char=Serial.read();
      while(received_char!=BANG && !inside_a_message && received_char>0){
        received_char=Serial.read();
        delay(DELAY);
      }
      if(received_char=='!'){
        inside_a_message=true;
      }
      if(received_char<0||!inside_a_message){
        continue;
      }
      inside_a_message=true;
      if(received_char!=ASTERICK){
        command_char_array[index]=ARRAY_DELIMITER;
        inside_a_message=false;
        index=0;
        int command[16]={0};
        build_command_from_received_char(command,command_char_array);
        send_command(command);
        delay(DELAY);
      }
      delay(DELAY);
    }
  }
}

void build_command(int command[],char received_char_arr[]){
  char integer_builder_arr[INTEGER_BUILDER_ARR_SIZE]={0};
  int integer_builder_arr_index=0;
  int command_index=0;
  //Iterate through all elements in the array till you reach the array delimiter
  //Ignore the first index (0) since it is just a message predecessor (BANG)
  for(int i=1;received_char_arr[i]!=ARRAY_DELIMITER;i++){
    if(received_char_arr[i]!=SPACE_CHARACTER){
      integer_builder_arr[integer_builder_arr_index++]=received_char_arr[i];
    }
    else if(received_char_arr[i]==SPACE_CHARACTER){
      //add a NULL character to the end of 'integer_builder_arr' and return the corresponding integer using atoi() function
      integer_builder_arr[integer_builder_arr_index]=NULL;
      int integer_value=atoi(integer_builder_arr);
      //append the 'command' array
      command[command_index++]=integer_value;
      //set integer_builder_arr_index to 0 and ree-initialize the corresponding array to all zeros
      integer_builder_arr_index=0;
      for(int x=0;x<INTEGER_BUILDER_ARR_SIZE;x++){
        integer_builder_arr[x]=0;
      }
    }
    else{
      Serial.println("This should never happen. Something terrible happened!");
    }
  }
  //add a NULL character to the end of 'integer_builder_arr' and return the corresponding integer using the atoi() function
  integer_builder_arr[integer_builder_arr_index]=NULL;
  int integer_value=atoi(integer_builder_arr);
  //append the command array
  command[command_index++]=integer_value;
  command[command_index]=ARRAY_DELIMITER;
}

void send_command(int command[]){
  char command_target_id=command[0];
  switch(command_target_id){
    case(MOTOR):
      int leftSpeed=command[1];
      int rightSpeed=command[2];
      leftPIDSetPoint=leftSpeed;
      rightPIDSetPoint=rightSpeed;
      break;
  }
}

void setLeftspeed(int duty){
  if(duty>255 || duty<-255)
    return;
  if(duty<0){
    analogWrite(LEFTMOTOR_PWM,(-duty));
    digitalWrite(LEFTMOTOR_FWD,LOW);
  }
  else{
    analogWrite(LEFTMOTOR_PWM,duty);
    digitalWrite(LEFTMOTOR_FWD,HIGH);
  }
}

void setRightspeed(int duty){
  if(duty>255 || duty<-255)
    return;
  if(duty<0){
    analogWrite(RIGHTMOTOR_PWM,(-duty));
    digitalWrite(RIGHTMOTOR_FWD,LOW);
  }
  else{
    analogWrite(RIGHTMOTORPWM,duty);
    digitalWrite(RIGHTMOTOR_FWD,HIGH);
  }
}

void cruiseControl(int targetSpeed){
  if(millis()-speedSampleTime>speedSampleInterval){
    speedSampleTime=millis();
    //Calculating Speed
    leftSpeed=leftEncoder-previousLeftEncoder;
    leftPIDInput=leftSpeed;
    rightSpeed=rightEncoder-previousRightEncoder;
    rightPIDInput=rightSpeed;
  }
  leftPID.Compute();
  if(leftPIDSetPoint==0)
    setLeftspeed(0);
  else
    setLeftspeed(leftPIDOutput);
  rightPID.Compute();
  if(rightPIDSetPoint==0)
    setRightspeed(0);
  else
    setRightspeed(rightPIDOutput);
}

void leftEncoder(){
  leftEncoder++;
}

void rightEncoder(){
  rightEncoder++;
}

void getObstacleDistance(int pin){
  int raw=analogRead(pin);
  if(raw>600)
    raw=600;
  float calc=270.0/(float)raw*1024.0/5.0;
  if(calc<100.0)
    return 100.0
  else if(calc>100.0)
    return 600.0
  else
    return calc;
}

int readAnalogMultiplexer(byte mux_pin){
  mux_pin=constrain(mux_pin,0,7);
  if(mux_pin & 0x01)
    digitalWrite(mux_s0,HIGH);
  else
    digitalWrite(mux_s0,LOW);
  if(mux_pin & 0x02)
    digitalWrite(mux_s1,HIGH);
  else
    digitalWrite(mux_s1,LOW);
  if(mux_pin & 0x03)
    digitalWrite(mux_s2,HIGH);
  else
    digitalWrite(mux_s2,LOW);
  return analogRead(mux_Signal);
}
