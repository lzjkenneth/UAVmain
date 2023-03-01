//
//  UAVionics example codes
//  TB6612FNG motor driver
//

// Left motor = Motor A / Motor 1
// Right motor = Motor B / Motor 2
// Motor controller pins, keep in mind the PWM defines must be on PWM pins
#define APWM           32
#define MA2            33
#define MA1            25
#define STANDBY        26
#define MB1            27
#define MB2            14
#define BPWM           12


// conversion of motor direction to integers
int FORWARD = 0;
int BACKWARD = 1;
int RELEASE = 2;


// register motor direction for counter
int direction1 = RELEASE;
int direction2 = RELEASE;


// control variable for motor speed
int PWM_val1 = 0;
int PWM_val2 = 0;


// set pinModes of defined motor controller input pins
void setupMotors()
{ // Setup motor pins and initialize motors

  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(APWM, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);
  pinMode(BPWM, OUTPUT);
  pinMode(STANDBY, OUTPUT);

  digitalWrite(STANDBY, HIGH); // HIGH if controlling motor

  // initialize motor speeds to 0, ranges from 0-255 on pwm pins
  motor1setSpeed(0);
  motor2setSpeed(0);

  // initialize motor to RELEASE (brake) mode, can be set to FORWARD and BACKWARD
  motor1run(RELEASE);
  motor2run(RELEASE);
}


// function to set motor1's direction
void motor1run(int direct)
{ // set pin logic for motor A direction
  if (direct == FORWARD)
  {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MA1, HIGH);
    digitalWrite(MA2, LOW);
  }
  else if (direct == BACKWARD)
  {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MA1, LOW);
    digitalWrite(MA2, HIGH);
  }
  else if (direct == RELEASE)
  {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MA1, HIGH);
    digitalWrite(MA2, HIGH);
  }
}


// function to set motor2's direction
void motor2run(int direct)
{ // set pin logic for motor B direction
  if (direct == FORWARD)
  {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MB1, LOW);
    digitalWrite(MB2, HIGH);
  }
  else if (direct == BACKWARD)
  {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MB1, HIGH);
    digitalWrite(MB2, LOW);
  }
  else if (direct == RELEASE)
  {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MB1, HIGH);
    digitalWrite(MB2, HIGH);
  }
}


// function to set motor1's speed
void motor1setSpeed(int Speed)
{ // set pin pwm for motor A direction
  analogWrite(APWM, Speed);
}

// function to set motor2's speed
void motor2setSpeed(int Speed)
{ // set pin pwm for motor B direction
  analogWrite(BPWM, Speed);
}



void setup() {
  // put your setup code here, to run once:

  // run function to initialize motor pins and states
  setupMotors();
}

void loop() {
  // put your main code here, to run repeatedly:

  // set motor direction to forward for Motor A and B
  direction1 = FORWARD;
  direction2 = FORWARD;

  // set motor speed to 150 for Motor A and B
  PWM_val1 = 150;
  PWM_val2 = 150;

  // send motor variables to motor functions
  motor1run(direction1);
  motor2run(direction2);
  motor1setSpeed(abs(PWM_val1));
  motor2setSpeed(abs(PWM_val2));


  // wait for 3 seconds
  delay(3000);


  // set motor direction to release for Motor A and B
  direction1 = RELEASE;
  direction2 = RELEASE;

  // set motor speed to 150 for Motor A and B
  PWM_val1 = 0;
  PWM_val2 = 0;

  // send motor variables to motor functions
  motor1run(direction1);
  motor2run(direction2);
  motor1setSpeed(abs(PWM_val1));
  motor2setSpeed(abs(PWM_val2));


  // wait for 3 seconds
  delay(3000);


  // set motor direction to release for Motor A and B
  direction1 = BACKWARD;
  direction2 = BACKWARD;

  // set motor speed to 150 for Motor A and B
  PWM_val1 = 150;
  PWM_val2 = 150;

  // send motor variables to motor functions
  motor1run(direction1);
  motor2run(direction2);
  motor1setSpeed(abs(PWM_val1));
  motor2setSpeed(abs(PWM_val2));

  // wait for 3 seconds
  delay(3000);

  
}
