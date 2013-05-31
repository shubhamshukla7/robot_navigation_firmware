#include <PID_v1.h> // http://playground.arduino.cc/Code/PIDLibrary
#include <PinChangeInt.h> // http://code.google.com/p/arduino-pinchangeint/wiki

// Some bluetooth messaging protocol constants
#define INTEGER_BUILDER_ARR_SIZE 8
#define ARRAY_DELIMITER 127
#define SPACE_CHARACTER ' '
#define MOTOR 0
#define ASTERICK 42
#define BANG 33
#define COMMAND_CHAR_ARRAY 64
#define DELAY_MILLISEC 5

// PID constants
#define const_p 8.0
#define const_i 32.0
#define const_d 0.1

// encoders
const byte pinQA_I = 2;
const byte pinQA_Q = 4;
const byte pinQB_I = 3;
const byte pinQB_Q = 5;

// motors
const byte pinAen = 11;
const byte pinAphase = 6;
const byte pinBen = 10;
const byte pinBphase = 12;

// analog multiplexer
const byte aMuxS0 = 7;
const byte aMuxS1 = 8;
const byte aMuxS2 = 9;
//const byte aMuxS4 = ?
const byte aMuxSig = A3;

double LeftPIDInput, LeftPIDOutput, LeftPIDSetpoint;
double RightPIDInput, RightPIDOutput, RightPIDSetpoint;
PID leftPID(&LeftPIDInput, &LeftPIDOutput, &LeftPIDSetpoint, const_p, const_i, const_d, DIRECT);
PID rightPID(&RightPIDInput, &RightPIDOutput, &RightPIDSetpoint, const_p, const_i, const_d, DIRECT);

volatile byte x, px, y, py;
volatile byte x2, px2, y2, py2;
volatile int LeftEnc = 0;
volatile int RightEnc = 0;
unsigned long msgTime = 0;

unsigned long speedSampleTime = 0; // keeping track of when to do speed calculation
const unsigned long speedSamplePeriod = 200; // in milliseconds

int prevLeftEnc = 0, prevRightEnc = 0;
int LeftSpeed, RightSpeed;

void setup() {
  pinMode(pinQA_I, INPUT );
  pinMode(pinQA_Q, INPUT );
  pinMode(pinQB_I, INPUT );
  pinMode(pinQB_Q, INPUT );
  pinMode(13, OUTPUT);
  x = digitalRead(pinQA_I); // read these to get the initial states for the ISRs
  y = digitalRead(pinQA_Q);
  x2 = digitalRead(pinQB_I);
  y2 = digitalRead(pinQB_Q);
  
  PCintPort::attachInterrupt(pinQA_I, &ISR_A_I, CHANGE);
  PCintPort::attachInterrupt(pinQA_Q, &ISR_A_Q, CHANGE);
  PCintPort::attachInterrupt(pinQB_I, &ISR_B_I, CHANGE);
  PCintPort::attachInterrupt(pinQB_Q, &ISR_B_Q, CHANGE);
  
  pinMode( aMuxS0, OUTPUT);
  pinMode( aMuxS1, OUTPUT);
  pinMode( aMuxS2, OUTPUT);
  //pinMode( aMuxS3, OUTPUT);
  pinMode( aMuxSig, INPUT);
  
  pinMode(pinAphase, OUTPUT);
  pinMode(pinBphase, OUTPUT);
  pinMode(pinAen, OUTPUT);
  pinMode(pinBen, OUTPUT);
  
  //attachInterrupt(0, left_encoder_ISR, CHANGE); // we test the value of the pin in the ISR
  //attachInterrupt(1, right_encoder_ISR, CHANGE); // ditto
  
  LeftPIDSetpoint = 0;
  RightPIDSetpoint = 0;
  leftPID.SetOutputLimits(-255, 255);
  rightPID.SetOutputLimits(-255, 255);
  leftPID.SetSampleTime(speedSamplePeriod);
  rightPID.SetSampleTime(speedSamplePeriod);
  leftPID.SetMode(AUTOMATIC); // turn PID on
  rightPID.SetMode(AUTOMATIC);
  
  Serial.begin( 115200 );
}

/**
 * Actual command: "!2 34 56*" is passed as:
 * Input character array = {'!', '2', ' ', '3', '4', ' ', '5', '6', ARRAY_DELIMITER }
 * Function builds the output integer command = {2, 34, 56, ARRAY_DELIMITER}
 */
void build_command_from_received_char(int command[], char received_char_arr[]) {
  
  char integer_builder_arr[INTEGER_BUILDER_ARR_SIZE] = {0};
  int integer_builder_arr_index = 0;
  
  int command_index = 0;
  
  /* 
   * Iterate through all elements in the array till you reach ARRAY_DELIMITER
   * Ignore the first index (index 0) since it is just a message predecessor (BANG)
   */
  for(int i=1; received_char_arr[i] != ARRAY_DELIMITER; i++) {

    if(received_char_arr[i] != SPACE_CHARACTER) {
      integer_builder_arr[integer_builder_arr_index++] = received_char_arr[i];
    }
    else if(received_char_arr[i] == SPACE_CHARACTER){
      // add a NULL character to the end of 'integer_builder_arr' and return the corresponding integer using atoi() function.
      integer_builder_arr[integer_builder_arr_index] = NULL;
      int integer_value = atoi(integer_builder_arr);
      
      // append the 'command' array
      command[command_index++] = integer_value;
      
      // set integer_builder_arr_index to 0 and re-initilize the correspondsing array to all zeros.
      integer_builder_arr_index = 0;
      for(int x=0;x<INTEGER_BUILDER_ARR_SIZE; x++) {
        integer_builder_arr[x] = 0;
      }
    }
    else {
      Serial.print("This should never happen. Something terrible happened!");
    }
  }
  // add a NULL character to the end of 'integer_builder_arr' and return the corresponding integer using atoi() function.
  integer_builder_arr[integer_builder_arr_index] = NULL;
  int integer_value = atoi(integer_builder_arr);
      
  // append the 'command' array
  command[command_index++] = integer_value;
  command[command_index] = ARRAY_DELIMITER;  
}

void send_command_to_robot(int command[]) {
  char command_target_id = command[0];
  switch(command_target_id) {
  case(MOTOR):
    int left_speed = command[1]; //between -255 and +255.
    int right_speed = command[2];
    Serial.print("Left speed::: ");
    Serial.println(left_speed);
    Serial.print("Right speed::: ");
    Serial.println(right_speed);
    LeftPIDSetpoint = left_speed;
    RightPIDSetpoint = right_speed;
    break;
  }
}

// command a motor speed with a signed integer 
void set_left_motor_speed( int spd)
{
  if( spd == 0)
    {
      analogWrite(pinAen, 0);
    }
  else if( spd < 0 )
    {
      digitalWrite(pinAphase, LOW);
      analogWrite(pinAen, (-spd) );
    }
  else // speed > 0
    {
      digitalWrite(pinAphase, HIGH);
      analogWrite(pinAen, spd );
    }
}

// command a motor speed with a signed integer 
void set_right_motor_speed( int spd)
{
  //spd = constrain( spd, -255, 255);
  if( spd == 0)
    {
      analogWrite(pinBen, 0);
    }
  else if( spd < 0 )
    {
      digitalWrite(pinBphase, LOW);
      analogWrite(pinBen, (-spd) );
    }
  else // speed > 0
    {
      digitalWrite(pinBphase, HIGH);
      analogWrite(pinBen, spd );
      }
}

void loop()
{
  if( millis() - speedSampleTime > speedSamplePeriod ) // every so often
  {
    speedSampleTime = millis();
    
    // first calculate speed
    LeftSpeed = LeftEnc - prevLeftEnc;
    prevLeftEnc = LeftEnc; // you could lose counts here
    LeftPIDInput = LeftSpeed;
    
    // Right motor
    RightSpeed = RightEnc - prevRightEnc;
    prevRightEnc = RightEnc; // you could lose counts here
    RightPIDInput = RightSpeed;
  }
  
  // this can be called just as often as we like
  leftPID.Compute();
  if( LeftPIDSetpoint == 0)
    SetLeftMotorSpeed( 0 );
  else
    SetLeftMotorSpeed( LeftPIDOutput );
  rightPID.Compute();
  if( RightPIDSetpoint == 0)
    SetRightMotorSpeed( 0 );
  else
    SetRightMotorSpeed( RightPIDOutput );

  /*
  if( millis() - msgTime > 250 )
  {
    msgTime = millis();
    Serial.print( "LE "); Serial.print( LeftEnc); Serial.print( " " );
    Serial.print( "RE " ); Serial.print( RightEnc ); Serial.print( " " );
    Serial.print( "LS " ); Serial.print( LeftSpeed ); Serial.print( " " );
    Serial.print( "RS " ); Serial.print( RightSpeed ); Serial.print( " " );
    Serial.print( "LSo " ); Serial.print( LeftPIDOutput ); Serial.print( " " );
    Serial.print( "RSo " ); Serial.print( RightPIDOutput ); Serial.print( " " );
    Serial.print( "LKp " ); Serial.print( leftPID.GetKp() ); Serial.print( " " );
    Serial.print( "LKi " ); Serial.print( leftPID.GetKi() ); Serial.print( " " );
    Serial.print( "LKd " ); Serial.print( leftPID.GetKd() ); Serial.print( " " );

    Serial.print( "Di0 " ); Serial.print( ReadSharpSensor(A0) ); Serial.print( " " );
    Serial.print( "Di1 " ); Serial.print( ReadSharpSensor(A1) ); Serial.print( " " );
    Serial.print( "Di2 " ); Serial.print( ReadSharpSensor(A2) ); Serial.print( " " );
    
    Serial.print( "Rfl ");
    for( int i = 0; i < 8; i++)
    {
      Serial.print( ReadAMux(i) ); Serial.print( " " );
    }
    
    Serial.print( "\r\n");
  }
  */

  // Sample message: "!0 14 15*"
  char command_char_array[COMMAND_CHAR_ARRAY] = {0};
  int index = 0;
  bool inside_a_message = false;
  if( Serial.available() > 0 )
  {
    // As long as there is something on the serial line, keep reading it (It has to be the message from the phone)
    while(Serial.available() > 0) {
      
      char received_char = Serial.read();
      // Check if it is the start of a new message
      while(received_char != BANG && !inside_a_message && received_char > 0) {
        received_char = Serial.read();
        // wait for a bit for the next item on the serial line to show up...
        delay(DELAY_MILLISEC);
      }
      
      if(received_char == '!') {
        inside_a_message = true;
      }
      
      if(received_char < 0 || !inside_a_message) {
        continue;
      }
      
      inside_a_message = true;

      if(received_char != ASTERICK) {
        command_char_array[index++] = received_char;
      }
      else { // if it is an ARRAY_DELIMITER
        command_char_array[index] = ARRAY_DELIMITER;
        inside_a_message = false;

        index = 0;
        int command[16] = {0};
        build_command_from_received_char(command, command_char_array);
        for(int x = 0 ; command[x-1] != ARRAY_DELIMITER; x++) {
          Serial.print(x);
          Serial.print(": ");
          Serial.print((int)command[x]);
          Serial.print(" (");
          Serial.print(command[x]);
          Serial.println(")");
        }
        send_command_to_robot(command);
        delay(DELAY_MILLISEC);
      }
      delay(DELAY_MILLISEC);
    }
    
    
    
 }
}

// left encoder interrupt
void ISR_A_I()
{
  px = x;
  x = digitalRead(pinQA_I);
  //y = digitalRead(pinQA_Q);
  
  digitalWrite( 13, y); // debugging LED
  
  if( px == LOW && x == HIGH ) {
    // x high transition
    if( y ) LeftEnc ++;
    else LeftEnc --;
  }
  
  if( px == HIGH && x == LOW ) {
    // x low transition
    if( y ) LeftEnc --;
    else LeftEnc ++;
  }
}

void ISR_A_Q()
{
  //x = digitalRead(pinQA_I);
  py = y;
  y = digitalRead(pinQA_Q);
  
  if( py == LOW && y == HIGH ) {
    // y high transition
    if( x ) LeftEnc --;
    else LeftEnc ++;
  }
  
  if( py == HIGH && y == LOW ) {
    // y low transision
    if( x ) LeftEnc ++;
    else LeftEnc --;
  }
}

// right encoder interrupt
void ISR_B_I()
{
  px2 = x2;
  x2 = digitalRead(pinQB_I);
  //y2 = digitalRead(pinQB_Q);
  
  if( px2 == LOW && x2 == HIGH ) {
    // x high transition
    if( y2 ) RightEnc ++;
    else RightEnc --;
  }
  
  if( px2 == HIGH && x2 == LOW ) {
    // x low transition
    if( y2 ) RightEnc --;
    else RightEnc ++;
  }
}

void ISR_B_Q()
{
  //x2 = digitalRead(pinQB_I);
  py2 = y2;
  y2 = digitalRead(pinQB_Q);
  
  if( py2 == LOW && y2 == HIGH ) {
    // y high transition
    if( x2 ) RightEnc --;
    else RightEnc ++;
  }
  
  if( py2 == HIGH && y2 == LOW ) {
    // y low transision
    if( x2 ) RightEnc ++;
    else RightEnc --;
  }
}

// command a motor speed with a signed integer
void SetLeftMotorSpeed( int spd)
{
  if( spd == 0)
  {
    analogWrite(pinAen, 0);
  } else if( spd < 0 )
  {
    digitalWrite(pinAphase, LOW);
    analogWrite(pinAen, (-spd) );
  } else // speed > 0
  {
    digitalWrite(pinAphase, HIGH);
    analogWrite(pinAen, spd );
  }
}
  
// command a motor speed with a signed integer
void SetRightMotorSpeed( int spd)
{
  //spd = constrain( spd, -255, 255);
    
  if( spd == 0)
  {
    analogWrite(pinBen, 0);
  } else if( spd < 0 )
  {
    digitalWrite(pinBphase, LOW);
    analogWrite(pinBen, (-spd) );
  } else // speed > 0
  {
    digitalWrite(pinBphase, HIGH);
    analogWrite(pinBen, spd );
  }
}

// measure the voltage of a sharp GP2Y0A21 IR distance sensor and return the distance in mm
float ReadSharpSensor( int pin)
{
  int raw = analogRead(pin);
  if( raw > 600)
    raw = 600;
  float calc = 270.0 / (float) raw * 1024.0 / 5.0;
  if( calc < 100.0 )
    return 100.0;
  else if( calc > 600.0 )
    return 600.0;
  else
    return calc;
}

// switch the analog mux and sample the attached analog port
int ReadAMux( byte mux_pin)
{
  mux_pin = constrain(mux_pin, 0, 7); // we currently only support the first three mux ports
  
  if(mux_pin & 0x01) digitalWrite( aMuxS0, HIGH); else digitalWrite( aMuxS0, LOW);
  if(mux_pin & 0x02) digitalWrite( aMuxS1, HIGH); else digitalWrite( aMuxS1, LOW);
  if(mux_pin & 0x04) digitalWrite( aMuxS2, HIGH); else digitalWrite( aMuxS2, LOW);
  //digitalWrite( aMuxS0, mux_pin & 0x01);
  //digitalWrite( aMuxS1, mux_pin & 0x02);
  //digitalWrite( aMuxS2, mux_pin & 0x04);
  //digitalWrite( aMuxS3, mux_pin & 0x08);
  
  return analogRead( aMuxSig);
}



