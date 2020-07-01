#include <Servo.h>

#include <Servo.h>
Servo LeftServo;
Servo RightServo;

int RightResistanceAngle = 0;
int LeftResistanceAngle = 0;

volatile unsigned int LeftWheel = 0;
volatile unsigned int RightWheel = 0;

//rotary encoder pin setup
int rotaryLeft1 = 18;
int rotaryLeft2 = 19;
int rotaryRight1 = 20;
int rotaryRight2 = 21;


//Linear actuator pin setup
int ActuatorEn = 2;
int ActuatorLeftRaise = 22;
int ActuatorLeftLower = 23;
int ActuatorRightRaise = 24;
int ActuatorRightLower = 25;

//Resistance system setup, use 1 and two, one high one low, for the first output, etc.
int leftPWM2 = 8;
int leftPWM1 = 13;
int leftIn1 = 12;
int leftIn2 = 11;
int leftIn3 = 10;
int leftIn4 = 9;
int rightPWM2 = 2;
int rightPWM1 = 7;
int rightIn1 = 6;
int rightIn2 = 5;
int rightIn3 = 4;
int rightIn4 = 3;



String  inputString = "";
int inputint;
boolean stringComplete = false;

unsigned long timeofexecution;
unsigned long currenttime;

Servo LeftResistance;
Servo RightResistance;
void setup() {
  LeftResistance.attach(52);
  RightResistance.attach(53);
  
  LeftServo.attach(12); 
  RightServo.attach(6); 
  Serial.begin(9600);
  inputString.reserve(200);

  
  //set up pins for the rotary encoder
  pinMode(rotaryLeft1, INPUT);
  pinMode(rotaryLeft2, INPUT);
  pinMode(rotaryRight1, INPUT);
  pinMode(rotaryRight2, INPUT);

  //set up pins for the linear actuator
  pinMode(ActuatorEn, OUTPUT);
  pinMode(ActuatorLeftRaise, OUTPUT);
  pinMode(ActuatorLeftLower, OUTPUT);
  pinMode(ActuatorRightRaise, OUTPUT);
  pinMode(ActuatorRightLower, OUTPUT);

  //set up pins for the resistance system
  pinMode(leftPWM2, OUTPUT);
  pinMode(leftPWM1, OUTPUT);
  pinMode(leftIn1, OUTPUT);
  pinMode(leftIn2, OUTPUT);
  pinMode(leftIn3, OUTPUT);
  pinMode(leftIn4, OUTPUT);
  pinMode(rightPWM2, OUTPUT);
  pinMode(rightPWM1, OUTPUT);
  pinMode(rightIn1, OUTPUT);
  pinMode(rightIn2, OUTPUT);
  pinMode(rightIn3, OUTPUT);
  pinMode(rightIn4, OUTPUT);


  digitalWrite(rotaryLeft1, HIGH);
  digitalWrite(rotaryLeft2, HIGH);
  digitalWrite(rotaryRight1, HIGH);
  digitalWrite(rotaryRight2, HIGH);

  //set up interrupts for the rotary encoders
  attachInterrupt(digitalPinToInterrupt(rotaryLeft1), left1, RISING);
  attachInterrupt(digitalPinToInterrupt(rotaryRight1), right1, RISING);
  attachInterrupt(digitalPinToInterrupt(rotaryLeft2), left2, RISING);
  attachInterrupt(digitalPinToInterrupt(rotaryRight2), right2, RISING);

}

void loop() {
  //put your main code here, to run repeatedly:
   LeftResistance.write(LeftResistanceAngle);
  RightResistance.write(RightResistanceAngle);
 Serial.print(LeftWheel);
 Serial.print(" : ");
  Serial.print(RightWheel);
  Serial.println();
  
  if(stringComplete) {
    //a full command has been input, time to do something with it!
    
    String temp = inputString;
    temp.remove(0,2);
    inputint = temp.toInt();
    Serial.println(inputString);
    Serial.println(inputint);
    if(inputString.indexOf("Hello") > -1)
    {
      Serial.println("Why Hello there");
    }
    if(inputString.indexOf("lagtest") > -1)
    {
      Serial.println("lagtestreply");
    }
    else if(inputString.indexOf("AL") > -1)
    {
      //Linear actuator align left
      moveActuators(inputint, true, false);
    }
    else if(inputString.indexOf("AR") > -1)
    {
      //Linear actuator align Right
      moveActuators(inputint, false, true);
    }
    else if (inputString.indexOf("MA") > -1)
    {
      moveActuators(inputint, true, true);
    }
    else if(inputString.indexOf("RR") > -1)
    {
      //resistance Right
      resistance(inputint, false);
    }
    else if(inputString.indexOf("RL") > -1)
    {
      //resistance left
      resistance(inputint, true);
    }
    
    inputString = "";
    stringComplete = false;
  }
}

//the interrupt methods called when there is a change in the position of the wheels
void left1() {
  if (digitalRead(rotaryLeft2) == LOW) {
    LeftWheel++;
  } else {
    LeftWheel--;
  }
}
void left2() {
  if (digitalRead(rotaryLeft1) == LOW) {
    LeftWheel--;
  } else {
    LeftWheel++;
  }
}
void right1() {
  if (digitalRead(rotaryRight2) == LOW) {
    RightWheel++;
  } else {
    RightWheel--;
  }
}
void right2() {
  if (digitalRead(rotaryRight1) == LOW) {
    RightWheel--;
  } else {
    RightWheel++;
  }
}

bool moveActuators(int input, bool l, bool r)
{
  if(input > 0)
  {
    if(l == true)
    {
    digitalWrite(ActuatorLeftRaise, LOW);
    digitalWrite(ActuatorLeftLower, HIGH);
    }
    if(r == true)
    {
    digitalWrite(ActuatorRightRaise, HIGH);
    digitalWrite(ActuatorRightLower, LOW);
    }
  }
  else
  {
    if(l == true)
    {
    digitalWrite(ActuatorLeftLower, LOW);
    digitalWrite(ActuatorLeftRaise, HIGH);
    }
    if(r == true)
    {
    digitalWrite(ActuatorRightLower,HIGH);
    digitalWrite(ActuatorRightRaise, LOW);
    }
    input = input *-1;
  }
  timeofexecution = millis();
  while(millis() < timeofexecution + input)
  {
  }
    digitalWrite(ActuatorLeftLower, LOW);
    digitalWrite(ActuatorLeftRaise, LOW);
    digitalWrite(ActuatorRightLower,LOW);
    digitalWrite(ActuatorRightRaise, LOW);
    return true;
}
bool resistance(int input, bool side)
{ 
  Serial.println("Entered resistance");
  float minval;
  float maxval;
  float difference;
  float temp;
  switch(side)
  {
    case true: //left side
    
    minval = 50;
    maxval = 80;
    difference = maxval-minval;
    difference = (difference * ((float) input / 100) + minval);
    LeftResistanceAngle = difference;
      break;

    case false: //right side
    minval = 65;
    maxval = 124;
    difference = maxval-minval;
    difference = (difference * ((float) input / 100) + minval);
      RightResistanceAngle = difference;
      break;
  }
}

//if a serial command is given, it is read here
void serialEvent() {
  while(Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
