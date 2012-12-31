const int GoStraight = 0;
const int SlightLeft = 1;
const int SlightRight = 2;
const int TurnLeft = 3;
const int TurnRight = 4;
const int Stop = 5;

const int LineRight_OTH = 400; // optical threshhold  Off 280  on 890
const int LineLeft_OTH = 400; // optical threshhold  off 175 on 870

const int TurnRight_OTH = 750; // optical threshhold  off 600 on  810
const int TurnLeft_OTH = 750; // optical threshhold off 550 on 810

const int SPEED = 200
const int RIGHT_SPEED = SPEED;
const int LEFT_SPEED = SPEED * 0.95;

int senseTurnRight = 0;
int senseTurnLeft = 0;
//motor A connected between A01 and A02
//motor B connected between B01 and B02

int STBY = 10; //standby

//Motor A
int PWMA = 3; //Speed control 
int AIN1 = 9; //Direction
int AIN2 = 8; //Direction

//Motor B
int PWMB = 5; //Speed control
int BIN1 = 11; //Direction
int BIN2 = 12; //Direction

int sensorRight = 0; 
int sensorLeft = 0;

int sensorTurnRight = 0;
int sensorTurnLeft = 0;

int currState; // What is the robot doing?
int currRand = 0;

void setup(){
  Serial.begin(115200); 
  Serial.println("Robot GO");
  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
}

void loop(){
  ReadSensors();
  //move(1, LEFT_SPEED, 1);
  //move(2, RIGHT_SPEED, 0);
  // return;

  if (sensorTurnLeft > TurnLeft_OTH) senseTurnLeft = 1;
  //else senseTurnLeft = 0;

  if (sensorTurnRight > TurnRight_OTH) senseTurnRight = 1;
  //else {senseTurnRight = 0;}

  if (sensorLeft < LineLeft_OTH && sensorRight < LineRight_OTH) {
    // both off
    if (currState != TurnLeft && currState != TurnRight)
    {
      currRand = random(0,2);
      if (currRand == 1) currState = TurnLeft;
      else currState = TurnRight;
    }
    if (senseTurnLeft == 1 && senseTurnRight == 0)
    {
      currState = TurnLeft;
      //senseTurnLeft = 0;
    }
    else if (senseTurnRight == 1 && senseTurnLeft == 0 )
    {
      currState = TurnRight;
      //senseTurnRight = 0;
    }
    else if (currRand == 1)

    {
      if (currState == TurnLeft) currState == TurnRight; 
      else currState = TurnLeft;
    }

    //    else currState = TurnRight;
    //    {
    //      senseTurnLeft = 0;
    //      senseTurnRight = 0;
    //    }

  }
  else if (sensorLeft > LineLeft_OTH && sensorRight < LineRight_OTH) {
    // left on
    currState = SlightRight;
  }
  else if (sensorLeft < LineLeft_OTH && sensorRight > LineRight_OTH) {
    // right on
    currState = SlightLeft;
  }
  else if (sensorLeft > LineLeft_OTH && sensorRight > LineRight_OTH && sensorTurnLeft > TurnLeft_OTH ) {
    // both on and Left Turn signal is tripped signaling that a T is present to the Left

    if (currRand == 1)
    {
      currState = TurnLeft;
      execute();
      delay(500-SPEED);
    }

  }

  else if (sensorLeft > LineLeft_OTH && sensorRight > LineRight_OTH && sensorTurnRight > TurnRight_OTH ) {
    // both on and Left Turn signal is tripped signaling that a T is present to the RIGHT

    if (currRand == 1)
    {
      currState = TurnRight;
      execute();
      delay(500-SPEED);
    }

  }

  else if (sensorLeft > LineLeft_OTH && sensorRight > LineRight_OTH) {
    // both on
    currState = GoStraight;
    currRand = random(0,15);
    senseTurnLeft = 0;
    senseTurnRight = 0;
  }

  // Serial.println(currState);
  execute();
  // move(1, 128, 1); //motor 1, half speed, right
  // move(2, 128, 0); //motor 2, half speed, right
}

void execute() {
  if (currState == GoStraight) {
    move(1, LEFT_SPEED, 1);
    move(2, RIGHT_SPEED, 0);
  }
  else if (currState == SlightLeft) {
    move(1, LEFT_SPEED, 1);
    move(2, RIGHT_SPEED * 0.8, 0);
  }
  else if (currState == SlightRight) {
    move(1, LEFT_SPEED *0.8, 1);
    move(2, RIGHT_SPEED, 0);
  }
  else if (currState == TurnLeft) {
    move(1, LEFT_SPEED, 1);
    move(2, RIGHT_SPEED, 1);
  }
  else if (currState == TurnRight) {
    move(1, LEFT_SPEED, 0);
    move(2, RIGHT_SPEED, 0);
  }
  else if (currState == Stop) {
    move(1, 0, 1);
    move(2, 0, 1);
  }
}

void FindLine(int dir) {

  //int dir = random(0,1);
  if (dir == 0) {
    // spin left

    move(1,LEFT_SPEED,1);
    move(2,RIGHT_SPEED,0);
  }
  else {
    // spin right
    move(1,LEFT_SPEED,0);
    move(2,RIGHT_SPEED,1);
  }
}

void move(int motor, int speed, int direction){
  //Move specific motor at speed and direction
  //motor: 0 for B 1 for A
  //speed: 0 is off, and 255 is full speed
  //direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if(motor == 1){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }
  else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

void stop(){
  //enable standby  
  digitalWrite(STBY, LOW);
}

const int ReadSensors(void){
  sensorRight = analogRead(A0); 
  sensorLeft = analogRead(A1);

  sensorTurnRight = analogRead(A4); 
  sensorTurnLeft = analogRead(A5);

  //  Serial.print(sensorRight);
  //  Serial.print(",");
  //  Serial.print(sensorLeft);
  //  Serial.print(",");
  //  Serial.print(sensorTurnRight);
  //  Serial.print(",");
  //  Serial.println(sensorTurnLeft);

  return 0;
}