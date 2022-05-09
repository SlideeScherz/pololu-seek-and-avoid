#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
Servo headServo; //Servo object
ButtonA buttonA;

//-----------------------DISTANCE CALCULATIONS-----------------------//
unsigned long currentMilllis;
unsigned long prevMillis;
const unsigned long PERIOD = 20;
long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;
//All calculations are done in cm

//CLICKS_PER_ROTATION * GEAR_RATIO =  Total # of clicks that happen in the encoder per wheel rotaiton
const float CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.81F;
//3.2 cm
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFRENCE = 10.0531;

//-----------------LOCATION INITS---------------------//

//location debugger
boolean locDebug = false;
boolean changeDebugger = false;

//right and left wheel constants
float b = 8.5;
float prevSl = 0.0F;
float prevSr = 0.0F;
float tempSl = 0.0F;
float tempSr = 0.0F;
float Sl = 0.0F;
float Sr = 0.0F;
float x = 0.0;
float y = 0.0;
float theta = 0;
float pos[3] = { x, y, theta };

//change constants
float travelDist = 0;
float sChange = 0;
float thetaChange = 0;
float xChange = 0;
float yChange = 0;

//500 100
// goals
const int NUMBER_OF_GOALS = 1;
//  float xGoals[NUMBER_OF_GOALS] = {270,300,0,60};
//  float yGoals[NUMBER_OF_GOALS] = {0,30, 0,0};
//  float xGoals[NUMBER_OF_GOALS] = {60, 80, -60,0};
//  float yGoals[NUMBER_OF_GOALS] = {0, 50, -30, 0};
float xGoals[NUMBER_OF_GOALS] = { 400 };
float yGoals[NUMBER_OF_GOALS] = { 100 };
int count = 0;
float xGoal = xGoals[count];
float yGoal = yGoals[count];
float endDist;
boolean isEndGame = false;

//Size of Way Points in cm
int leeWay = 3;

//distance from origin
double G_FROM_O = sqrt(sq(xGoal - x) + sq(y - yGoal));
float dist = G_FROM_O;
float slowDist = 40;
const double GO_DIST_FACTOR = G_FROM_O / 2;
//----------------------Ultra Sound------------------------------//
//Initialize UltraSound
const int ECHO_PIN = 4;
const int TRIG_PIN = 5;

//Ultrasonic Mic
const int MAX_DISTANCE = 80; //(MAX_DISTANCE cm / 2 meters)

// Determine the normalization factor based on the MAX_DISTANCE
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE = 10;

//Ultrasonic timing
unsigned long usCurrMil;
unsigned long usPrevMil;
const unsigned long US_PERIOD = 50; // Time to wait for 1st ultra sound to activate

//-------------------------Servo--------------------------------//
//debugger switch: HEAD_DEBUG
const boolean HEAD_DEBUG = false;

//Head Servo Timing
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 300;

//Head esrvo constants
const int HEAD_SERVO_PIN = 22;
const int NUM_HEAD_POSITIONS = 5;

//adjusted the angles so US was facing straight foward.at the middle element
//{140,130,120,110,100,25,25}
//{113,98,83}
//155 > x Looking Right (negative)
//155 < x Looking Left  (positive)
//Normal Array {115,135,155,175,195}
//Test{105,155,205}
const int HEAD_POSITITIONS[NUM_HEAD_POSITIONS] = { 50, 70, 90, 110, 130 };
int POSITION_MULTIPLIERS[NUM_HEAD_POSITIONS] = { .25, 1, 4, -1, -.25 };

//distances at the head positions
//{0,0,0,0,0,0,0}
//{0,0,0}
double DISTANCES[NUM_HEAD_POSITIONS] = { MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE };

//head servo data
boolean headDirectionClcokwise = true;
int currHeadPos = 0;

//head servo data
boolean headDirectionClockwise = true;

//------------------Motor--------------------------//
//debugger switch: Motor Debg
boolean MOTOR_DEBUG = false;

//Motor constants
const float MOTOR_BASE_SPEED = 100;
const int MOTOR_MIN_SPEED = 50;
const int MOTOR_MAX_SPEED = 150;

//start out with the MOTOR_BASE_SPEED
float leftSpeed = MOTOR_BASE_SPEED;
float rightSpeed = MOTOR_BASE_SPEED;

// Determine the normalization factor based on Motor_Based_Speed
const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 100;

//Motor timing
unsigned long motorCurrMil;
unsigned long motorPrevMil;
//Time to wait for adjusting the motor speed
const unsigned long MOTOR_PERIOD = 20;
//------------P & USdetection------------------//

//initializes so there is a value the first time moveHead runs
double newDist = MAX_DISTANCE;

//debugger switch: PID debugg
boolean PID_DEBUG = false;
boolean USD_DEBUG = true;

//USD constant
double kusd = 13;

//determines when an object is WAY too close
double closeObj = 1000;

//PID constants
double kp = 500;

//PID calulated values
double pro = 1;

//error
double err = 0;

double usd = 0;

//object aviodance constants
double usdetection;
double posMag;

//Ultra Sonic Detection
double detectionLevel;
double detectionLevels[NUM_HEAD_POSITIONS] = { 0, 0, 0, 0, 0 };

//limits
double detectionMax = 100;
double detectionMin = -100;

float magnitude;

//----------------------Set Up-------------------------------//
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  // init head postion
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(90);

  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);
  encoders.flipEncoders(true);

  //Prep US
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  delay(1000);
  buzzer.play("c32");
}

//Interprets the reading on the Ultra Sonic as a distance
double usReadCm() {
  usCurrMil = millis();
  if (usCurrMil > usPrevMil + US_PERIOD) {

    //Clears the TRIG_PIN (set low)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    //Sets the TRIG_PIN HIGH(Active) for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    //Reads the ECHO_PIN, returns the sound wave travel time in microseconds
    //note the durration (38000 microsec) that will allow for reading up max distance supported by the sensor
    double duration = pulseIn(ECHO_PIN, HIGH, 38000);

    //calc the distance
    double distance = duration * 0.034 / 2; //Time of flight equation: Speed of sound wave divided by 2

    //apply limits
    if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
    if (distance == 0) distance = MAX_DISTANCE;

    //update the prevMillis
    usPrevMil = usCurrMil;

    return distance;

  }//if
}//usReadCm

//moves the US mounted on the servo
void moveHead(double objDist) {
  headCm = millis();

  if (headCm > headPm + HEAD_MOVEMENT_PERIOD) {

    //position head to currrent postion in array
    headServo.write(HEAD_POSITITIONS[currHeadPos]);
    //adding the distance to the array
    DISTANCES[currHeadPos] = objDist;

    //head debug output
    if (HEAD_DEBUG) {
      Serial.print(currHeadPos);
      Serial.print(" - ");
      Serial.println(HEAD_POSITITIONS[currHeadPos]);
      printDist();
    }//if

    /**
       See next head posttion
       Moves Servo to the next head Position and changes directions when needed
    */
    if (headDirectionClockwise) {
      if (currHeadPos >= (NUM_HEAD_POSITIONS - 1)) {
        headDirectionClockwise = !headDirectionClockwise;
        currHeadPos--;
      }//if
      else {
        currHeadPos++;
      }
    }//if

    else {
      if (currHeadPos <= 0) {
        headDirectionClockwise = !headDirectionClockwise;
        currHeadPos++;
      }//if

      else {
        currHeadPos--;
      }
    }//else

    //Updates the USD here so usd only updates when there is a new reading from the US
    //Also allows this reading to be independent from Proportion
    usd = updateUSD(currHeadPos);

    //reset previous millis
    headPm = headCm;
  }//if
}//moveHead

//Prints the array of distnaces
void printDist() {
  Serial.print("{");
  //sizeof determines the size of bytes
  //A long is 4 bytes
  //Divide the bytes of the array by its data type to get the  array length
  for (int i = 0; i < sizeof(DISTANCES) / sizeof(long); i++)
  {
    Serial.print(" ");
    Serial.print(DISTANCES[i]);
    Serial.print(",");
  }//for
  Serial.print("}");
  Serial.println("");
}//printDist

void checkEncoders() {
  currentMilllis = millis();

  if (currentMilllis > prevMillis + PERIOD)
  {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    //Saves the previous values of Sl and Sr
    prevSl = Sl;
    prevSr = Sr;

    //Calculates the distance of the left and right wheel
    Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFRENCE);
    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFRENCE);

    tempSl = Sl - prevSl;
    tempSr = Sr - prevSr;

    prevLeft = countsLeft;
    prevRight = countsRight;
    prevMillis = currentMilllis;

    //udpates when the time period passes
    updateChange();
    pro = updateP();

    if (locDebug) {
      Serial.print("Left: ");
      Serial.print(Sl);
      Serial.print("Right: ");
      Serial.println(Sr);

      Serial.print("Temp Sl");
      Serial.println(tempSl);
      Serial.print("Temp Sr");
      Serial.println(tempSr);
      Serial.print("Travel Distance: ");
      Serial.println(travelDist);

      Serial.print("Goal(x,y): (");
      Serial.print(xGoal);
      Serial.print(",");
      Serial.print(yGoal);
      Serial.println(")");

      Serial.print("Current position (x,y, theta):  ");
      Serial.print("(");
      Serial.print(x);
      Serial.print(",");
      Serial.print(y);
      Serial.print(",");
      Serial.print(theta);
      Serial.println("%)");
      Serial.print("Distance from goal :");
      Serial.println(dist);
      Serial.println("############################################");
    }//if
  }//if
}//CheckEncoders

void updateChange() {
  //traveled
  travelDist = (tempSl + tempSr) / 2;

  thetaChange = (tempSr - tempSl) / b;

  xChange = travelDist * cos(theta + thetaChange / 2);
  yChange = travelDist * sin(theta + thetaChange / 2);
  //updates the current location
  x += xChange;
  y += yChange;
  theta += thetaChange;
  dist = sqrt(sq(xGoal - x) + sq(y - yGoal));


  if (changeDebugger) {
    Serial.print("sChange :");
    Serial.println(sChange);
    Serial.print("thetaChange :");
    Serial.println(thetaChange);
    Serial.print("xChange :");
    Serial.println(xChange);
    Serial.print("yChange :");
    Serial.println(yChange);
  }//if

}//updateChange

double getError() {
  float error = theta - atan2(yGoal - y, xGoal - x);
  return error;
}//getError

double updateP() {
  //positive err --> Left
  //negative err --> Right
  err = getError();

  //----------PROPORTION------//
  double proportional = kp * err;

  if (PID_DEBUG) {
    Serial.println("------PID-----");
    Serial.print("currAngle:");
    Serial.println(theta);
    Serial.print("P: ");
    Serial.println(proportional);
  }//if

  return proportional;
}

//returns the controllerOutput
double updateUSD(int currPos) {

  double updateValue;

  //detectionLevel = position magnitude * position multiplier
  detectionLevel = (MAX_DISTANCE - DISTANCES[currPos]) * POSITION_MULTIPLIERS[currPos];

  //Decides which direction the front will apply towards
  //Note to self calc the middpoint of the array later rather than hard coding it in
  if (currPos == 2) {
    closeObj = detectionLevel;


    //Determines which side has the most obsticles
    //Negative --> Go Right
    if (usdetection < 0)
      detectionLevels[currPos] = detectionLevel * -1;
    //Postive --> Go Left
    else {
      detectionLevels[currPos] = detectionLevel;
    }//else

  }//if
  //Usual Case --> calcualtes the rest of the postions
  else {
    detectionLevels[currPos] = detectionLevel;
  }//else

  //resets the usdetection
  usdetection = 0;
  //Calculates the new USD
  for (int i = 0; i < sizeof(detectionLevels) / sizeof(double); i++) {
    usdetection += detectionLevels[i];
  }//for


  if (USD_DEBUG) {
    Serial.print("Pos ");
    Serial.print(currPos);
    Serial.print(": ");
    Serial.println(detectionLevel);
    Serial.print("US detection:");
    Serial.println(usdetection);
    Serial.print("Object very close (when less than 50)");
    Serial.println(closeObj);
  }//if

  return usdetection * kusd;
}//Update PID

//Identifies when the robot reaches the goal
boolean isFinished() {
  boolean fin = false;
  boolean complete = false;
  if ((xGoal - leeWay <= x && xGoal + leeWay >= x) && (yGoal - leeWay <= y && yGoal + leeWay >= y))
    fin = true;

  if (fin) {
    count++;
    xGoal = xGoals[count];
    yGoal = yGoals[count];

    //Sets a new Max distance from the new way point
    G_FROM_O = sqrt(sq(xGoal - x) + sq(y - yGoal));

    if (count < NUMBER_OF_GOALS)
      buzzer.play("c32");

    if (count == NUMBER_OF_GOALS - 1)
      isEndGame = true;

    else if (count == NUMBER_OF_GOALS) {
      buzzer.play("c32");
      buzzer.play("abc");
      motors.setSpeeds(0, 0);
      complete = true;

    }//if
  }//if

  return complete;
}//isFinished

//Adjusts the motor speeds by taking the angle and x/y goals into consideration
void setMotors(double controllerOutput) {
  motorCurrMil = millis();

  if (motorCurrMil > motorPrevMil + MOTOR_PERIOD)
  {

    //check to see if the most current distnace measurement is less then / equal to MAX_DISTANCE
    //determine the magnitude of the distance by taking the difference (short distnace = high distance)
    //divide by the DISTANCE_FACTOR to ensure uniform response as MAX_DISTNACE changes
    //This maps the distnace(1 - MAX_RANGE) to 0-100 for the magnitude
    magnitude = (float)(G_FROM_O - dist) / GO_DIST_FACTOR;
    //ex: MAX_DISTANCE = 80, distnace = 40: 80 - 40 = 40/.8 = 50(midrange)
    //ex: MAX_DISTNACE = 160, distance = 40: 160- 40 = 120/1.6 = 75 (top 1/4)


    //too far left from target (go right)
    if (controllerOutput < 0) {
      //CO is negative
      leftSpeed = getWheel(controllerOutput, 1);
      rightSpeed = getWheel(controllerOutput, -1);
    }
    //too far right from target (go left)
    else {
      //CO is positive
      rightSpeed = getWheel(controllerOutput, -1);
      leftSpeed = getWheel(controllerOutput, 1);
    }//else

    //lower limit check
    if (leftSpeed < MOTOR_MIN_SPEED) leftSpeed = MOTOR_MIN_SPEED;
    if (rightSpeed < MOTOR_MIN_SPEED) rightSpeed = MOTOR_MIN_SPEED;
    //max limit check
    if (leftSpeed > MOTOR_MAX_SPEED) leftSpeed = MOTOR_MAX_SPEED;
    if (rightSpeed > MOTOR_MAX_SPEED) rightSpeed = MOTOR_MAX_SPEED;

    motors.setSpeeds(leftSpeed, rightSpeed);
    if (MOTOR_DEBUG) {
      Serial.print("Left: ");
      Serial.println(leftSpeed);
      Serial.print("Right: ");
      Serial.println(rightSpeed);
    }//if

    motorPrevMil = motorCurrMil;
  }//if
}//setMotors

//calculates the wheelspeed
double getWheel(double co, int sign) {
  //Multiple the magnitude by the MOTOR_FACTOR to map the magnitude range(0 - 100) at the motors
  //(0 - MOTOR_BASED_SPEED)
  //Added Min Speed as a constant to increase the speed
  double wheelSpeed = (MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR)) + (co * sign) + (MOTOR_MIN_SPEED);


  //Slows down when objects are in front of it
  if (DISTANCES[currHeadPos] < slowDist)
    wheelSpeed = wheelSpeed * (dist / slowDist);

  //slows the robot down when it is approaching the end goal
  //Wheel gradually slows down when the slow dist is reached
  if (dist < slowDist)
    wheelSpeed = wheelSpeed * (dist / slowDist);



  if (MOTOR_DEBUG) {
    Serial.println("Base, Mag, and factor");
    Serial.println((MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR)));
    Serial.println("pid output:");
    Serial.println(co * sign);
    Serial.println("True Wheel Speed:  ");
    Serial.println(wheelSpeed);
  }//if
  return wheelSpeed;
}//getWheel

void loop() {

  if (!isFinished()) {
    moveHead(newDist);
    checkEncoders();
    setMotors(pro + usd);
    newDist = usReadCm();
  }//if

  else {
    motors.setSpeeds(0, 0);

    buzzer.play("c34");
    delay(100000);
  }//else

  //Allows me to reset the location for debugging purposes
  if (buttonA.isPressed()) {
    usdetection = 0;
    Sl = 0;
    Sr = 0;
    x = 0;
    y = 0;
    theta = 0;
  }//if
}//loop