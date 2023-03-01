//
//  UAVionics example codes
//  TB6612FNG motor driver & HC020K speed sensor
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
int IRAM_ATTR FORWARD = 0;
int IRAM_ATTR BACKWARD = 1;
int IRAM_ATTR RELEASE = 2;

// register motor direction for counter
int IRAM_ATTR direction1 = FORWARD;
int IRAM_ATTR direction2 = FORWARD;

// control variable for motor speed
int PWM_val1 = 0;
int PWM_val2 = 0;

// counter variables
volatile long IRAM_ATTR posA = 0;          // encoder A counter
volatile long IRAM_ATTR posB = 0;          // encoder B counter

// Encoder pins
#define encodPinA      18     // encoder A pin
#define encodPinB      19     // encoder B pin



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





void IRAM_ATTR encoder1()
{ // count encoder ticks for motor A
  if (direction1 == FORWARD) {
    posA++;
  }
  if (direction1 == BACKWARD) {
    posA--;
  }
}

void IRAM_ATTR encoder2()
{ // count encoder ticks for motor B
  if (direction2 == FORWARD) {
    posB++;
  }
  if (direction2 == BACKWARD) {
    posB--;
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // configure encoder interrupts

  // turn on pullup resistors
  pinMode(encodPinA, INPUT_PULLUP);
  pinMode(encodPinB, INPUT_PULLUP);

  // call encoder1 function when encodPinA's logic state CHANGEs
  attachInterrupt(encodPinA, encoder1, CHANGE);
  attachInterrupt(encodPinB, encoder2, CHANGE);

  // run function to initialize motor pins and states
  setupMotors();
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("posA: " + String(posA) + "\t posB: " + String(posB));

  // change direction after two full rotations
  if (posA > 40)
    direction1 = BACKWARD;
  else if (posA < -40)
    direction1 = FORWARD;

  if (posB > 40)
    direction2 = BACKWARD;
  else if (posB < -40)
    direction2 = FORWARD;


  // set motor speed to 150 for Motor A and B
  PWM_val1 = 150;
  PWM_val2 = 150;

  // send motor variables to motor functions
  motor1run(direction1);
  motor2run(direction2);
  motor1setSpeed(abs(PWM_val1));
  motor2setSpeed(abs(PWM_val2));


  delay(100); //Small delay between loops

}
