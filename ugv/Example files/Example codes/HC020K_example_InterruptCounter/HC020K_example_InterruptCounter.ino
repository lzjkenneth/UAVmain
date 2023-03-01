//
//  UAVionics example codes
//  HC020K speed sensor
//



// conversion of motor direction to integers
int IRAM_ATTR FORWARD = 0;
int IRAM_ATTR BACKWARD = 1;
int IRAM_ATTR RELEASE = 2;

// register motor direction for counter
int IRAM_ATTR direction1 = FORWARD;
int IRAM_ATTR direction2 = FORWARD;

// counter variables
volatile long IRAM_ATTR posA = 0;          // encoder A counter
volatile long IRAM_ATTR posB = 0;          // encoder B counter

// Encoder pins
#define encodPinA      18     // encoder A pin
#define encodPinB      19     // encoder B pin

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
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("posA: " + String(posA) + "\t posB: " + String(posB));

  delay(100); //Small delay between loops

}
