/**
 * Name: pololu_seek_and_avoid.ino
 * Created: 5/9/2022 12:36:13 AM
 * Author: Team Five
 *
 * All distance calculations done in CM.
 * All timer data will be in milliseconds
 * Time and delay for ulatrasonic sending and receiving pings will be MICROSECONDS. See Docs
 * Any reference to headServo will be for the sweeping servo the ultrasonic is attached to. Not a wheel motor.
 */

#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
Servo headServo;
ButtonA buttonA;

// calculate euclidean distance
double eucDistance(double goalX, double goalY, double posX, double posY)
{
  return sqrt(sq(goalX - posX) + sq(goalY - posY));
}

// misc constants
const double SPEED_OF_SOUND = 0.034;

// pin assignments
const int TRIG_PIN = 5;
const int ECHO_PIN = 4;
const int HEAD_SERVO_PIN = 18;

// debug switches
const bool MOTOR_DEBUG = false;
const bool LOC_DEBUG = false;
const bool ENC_DEBUG = false;
const bool HEAD_DEBUG = false;
const bool PID_DEBUG = false;
const bool USD_DEBUG = true;

// scheduler intervals
const unsigned long MOTOR_PERIOD = 20;          // motor speed
const unsigned long ENC_PERIOD = 20;            // count encoders
const unsigned long US_PERIOD = 50;             // ultrasonic ping
const unsigned long HEAD_MOVEMENT_PERIOD = 300; // sweep head

// CLICKS_PER_ROTATION * GEAR_RATIO =  Total # of clicks that happen in the encoder per wheel rotaiton
const float CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFRENCE = 10.0531;

// localization
float x = 0.0, y = 0.0, theta = 0;

// encoder data

// encoder timers
unsigned long encT1, encT2;
long countsLeft = 0, countsRight = 0;
long prevLeft = 0, prevRight = 0;

// distance between left and right wheels
float b = 8.5;

// distance traveled by each wheel
float prevSl = 0.0F, prevSr = 0.0F;
float tempSl = 0.0F, tempSr = 0.0F;
float Sl = 0.0F, Sr = 0.0F;

float travelDist = 0;
float sChange = 0;

float xChange = 0, yChange = 0, thetaChange = 0;

//  goal data

const int NUMBER_OF_GOALS = 1;
//  float xGoals[NUMBER_OF_GOALS] = {270,300,0,60};
//  float yGoals[NUMBER_OF_GOALS] = {0,30, 0,0};
//  float xGoals[NUMBER_OF_GOALS] = {60, 80, -60,0};
//  float yGoals[NUMBER_OF_GOALS] = {0, 50, -30, 0};

float xGoals[NUMBER_OF_GOALS] = { 500 };
float yGoals[NUMBER_OF_GOALS] = { 100 };
int currentGoal = 0;
float xGoal = xGoals[currentGoal];
float yGoal = yGoals[currentGoal];
bool goalComplete = false;

// Size of Way Points in cm
int acceptedError = 3;

// distance from origin
double goalDistance = eucDistance(xGoal, yGoal, x, y);
float dist = goalDistance;
float slowDist = 40;
const double GO_DIST_FACTOR = goalDistance / 2;

// ultrasonic data

// ultrasonic farthest data we want to keep
const int MAX_DISTANCE = 80;

// deceleration dampening based on goals and objects ahead
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE = 10;

// ultrasonic timing
unsigned long usT1, usT2;

// headServo timers
unsigned long hsT1, hsT2;

const int NUM_HEAD_POSITIONS = 5;

// 155 > x Looking Right (negative)
// 155 < x Looking Left  (positive)
const int HEAD_POSITITIONS[NUM_HEAD_POSITIONS] = { 115, 135, 155, 175, 195 };

// repellant factor for obstacles
int POS_FACTOR[NUM_HEAD_POSITIONS] = { .25, 1, 4, -1, -.25 };

/**
 * @brief Distances of objects in head pos array
 * Init all to max so a false obstacle isnt detected
 */
double DISTANCES[NUM_HEAD_POSITIONS] = { MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE };

// head servo data

// index of position in HEAD_POSITITIONS array
int currHeadPos = 0;

// boolean flag indicating if sweeping left or right
bool sweepingClockwise = true;

double tempDistance = 0.0, tempDuration = 0.0;

// wheel motor data

const float MOTOR_BASE_SPEED = 150;

// limits
const int MOTOR_MIN_SPEED = 100, MOTOR_MAX_SPEED = 200;

// init to base speed
float leftSpeed = MOTOR_BASE_SPEED;
float rightSpeed = MOTOR_BASE_SPEED;

// spring / dampening factor for speed
const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 100;

// motor timers
unsigned long mT1, mT2;

// P & USdetection

// init to max so it doesnt think theres a false obsticle on start
double newDist = MAX_DISTANCE;

// USD constant
const double USD_FACTOR = 13;

// determines when an object is WAY too close
// TUNE too high ?
double closeObj = 1000;

// PID constants
const double KP_GAIN = 500;

// PID calulated values
double gain = 1;

// error
double err = 0;

double usd = 0;

// object aviodance constants
double usdetection;

// Ultra Sonic Detection
double detectionLevel;
double detectionLevels[NUM_HEAD_POSITIONS] = { 0, 0, 0, 0, 0 };

// limits
double detectionMax = 100;
double detectionMin = -100;

float magnitude;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(57600);

  // init head postion
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(185);

  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);
  encoders.flipEncoders(true);

  // Prep US
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  delay(1000);
  // buzzer.play("c32");
}

void loop()
{

  if (!isFinished())
  {
    moveHead(newDist);
    checkEncoders();
    setMotors(gain + usd);
    newDist = sendPing();
  }

  else
  {
    motors.setSpeeds(0, 0);
    // buzzer.play("c34");
    delay(100000);
  }
}

/**
 * @brief Send an ultrasonic ping and to read distance.
 * Calculate round trip time of flight at room temperature
 * @return double distance read in CM
 */
double sendPing()
{
  usT1 = millis();
  if (usT1 > usT2 + US_PERIOD)
  {

    // Clears the TRIG_PIN (set low)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Sets the TRIG_PIN HIGH(Active) for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
    // note the durration (38000 microsec) that will allow for reading up max distance supported by the sensor
    tempDuration = pulseIn(ECHO_PIN, HIGH, 38000);

    // calc the distance
    tempDistance = getTimeOfFlight(tempDuration);

    // apply limits
    if (tempDistance > MAX_DISTANCE)
      tempDistance = MAX_DISTANCE;

    // handle timeout
    else if (tempDistance == 0)
      tempDistance = MAX_DISTANCE;

    // reset timer
    usT2 = usT1;

    return tempDistance;
  }
}

double getTimeOfFlight(double pingDuration)
{
  return pingDuration * SPEED_OF_SOUND / 2;
}

// moves the US mounted on the servo
void moveHead(double objDist)
{
  hsT1 = millis();

  if (hsT1 > hsT2 + HEAD_MOVEMENT_PERIOD)
  {

    // position head to currrent postion in array
    headServo.write(HEAD_POSITITIONS[currHeadPos]);
    // adding the distance to the array
    DISTANCES[currHeadPos] = objDist;

    // head debug output
    if (HEAD_DEBUG)
      debugHeadServo("headServo,");

    /**
     * See next head posttion
     * Moves Servo to the next head Position and changes directions when needed
     */
    if (sweepingClockwise)
    {
      if (currHeadPos >= (NUM_HEAD_POSITIONS - 1))
      {
        sweepingClockwise = !sweepingClockwise;
        currHeadPos--;
      }
      else
      {
        currHeadPos++;
      }
    }

    else
    {
      if (currHeadPos <= 0)
      {
        sweepingClockwise = !sweepingClockwise;
        currHeadPos++;
      }

      else
      {
        currHeadPos--;
      }
    }

    // Updates the USD here so usd only updates when there is a new reading from the US
    // Also allows this reading to be independent from Proportion
    usd = updateUSD(currHeadPos);

    // reset previous millis
    hsT2 = hsT1;
  }
}

void checkEncoders()
{
  encT1 = millis();

  if (encT1 > encT2 + ENC_PERIOD)
  {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    // Saves the previous values of Sl and Sr
    prevSl = Sl;
    prevSr = Sr;

    // Calculates the distance of the left and right wheel
    Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFRENCE);
    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFRENCE);

    tempSl = Sl - prevSl;
    tempSr = Sr - prevSr;

    prevLeft = countsLeft;
    prevRight = countsRight;
    encT2 = encT1;

    // udpates when the time period passes
    updateChange();
    gain = updateP();

    if (ENC_DEBUG)
      debugEncoders("Encoders,");

    if (PID_DEBUG)
      debugPID("PID,");

    if (LOC_DEBUG)
      debugLocation("Location,");
  }
}

void updateChange()
{
  // traveled
  travelDist = (tempSl + tempSr) / 2;

  thetaChange = (tempSr - tempSl) / b;

  xChange = travelDist * cos(theta + thetaChange / 2);
  yChange = travelDist * sin(theta + thetaChange / 2);
  // updates the current location
  x += xChange;
  y += yChange;
  theta += thetaChange;
  dist = eucDistance(xGoal, yGoal, x, y);
}

double getError(double currentTheta)
{
  return currentTheta - atan2(yGoal - y, xGoal - x);
}

double updateP()
{
  // positive err --> Left
  // negative err --> Right
  err = getError(theta);

  return KP_GAIN * err;
  ;
}

// returns the controllerOutput
double updateUSD(int currPos)
{

  double updateValue = 0.0;

  // detectionLevel = position magnitude * position multiplier
  detectionLevel = (MAX_DISTANCE - DISTANCES[currPos]) * POS_FACTOR[currPos];

  // Decides which direction the front will apply towards
  // Note to self calc the middpoint of the array later rather than hard coding it in
  if (currPos == 2)
  {
    closeObj = detectionLevel;

    // Determines which side has the most obsticles
    // Negative --> Go Right
    if (usdetection < 0)
      detectionLevels[currPos] = detectionLevel * -1;
    // Postive --> Go Left
    else
    {
      detectionLevels[currPos] = detectionLevel;
    }
  }
  // Usual Case --> calcualtes the rest of the postions
  else
  {
    detectionLevels[currPos] = detectionLevel;
  }

  // resets the usdetection
  usdetection = 0;
  // Calculates the new USD
  for (int i = 0; i < sizeof(detectionLevels) / sizeof(double); i++)
  {
    usdetection += detectionLevels[i];
  }

  if (USD_DEBUG)
    debugUltrasonic("Ultrasonic,");

  return usdetection * USD_FACTOR;
}

// Identifies when the robot reaches the goal
bool isFinished()
{
  bool fin = false;
  bool complete = false;
  if ((xGoal - acceptedError <= x && xGoal + acceptedError >= x) && (yGoal - acceptedError <= y && yGoal + acceptedError >= y))
    fin = true;

  if (fin)
  {
    currentGoal++;
    xGoal = xGoals[currentGoal];
    yGoal = yGoals[currentGoal];

    // Sets a new Max distance from the new way point
    goalDistance = eucDistance(xGoal, yGoal, x, y);

    if (currentGoal < NUMBER_OF_GOALS)
      // buzzer.play("c32");

      if (currentGoal == NUMBER_OF_GOALS - 1)
        goalComplete = true;

      else if (currentGoal == NUMBER_OF_GOALS)
      {
        // buzzer.play("c32");
        motors.setSpeeds(0, 0);
        complete = true;
      }
  }
  return complete;
}

// Adjusts the motor speeds by taking the angle and x/y goals into consideration
void setMotors(double controllerOutput)
{
  mT1 = millis();

  if (mT1 > mT2 + MOTOR_PERIOD)
  {

    // check to see if the most current distnace measurement is less then / equal to MAX_DISTANCE
    // determine the magnitude of the distance by taking the difference (short distnace = high distance)
    // divide by the DISTANCE_FACTOR to ensure uniform response as MAX_DISTNACE changes
    // This maps the distnace(1 - MAX_RANGE) to 0-100 for the magnitude
    magnitude = (float)(goalDistance - dist) / GO_DIST_FACTOR;
    // ex: MAX_DISTANCE = 80, distnace = 40: 80 - 40 = 40/.8 = 50(midrange)
    // ex: MAX_DISTNACE = 160, distance = 40: 160- 40 = 120/1.6 = 75 (top 1/4)

    // too far left from target (go right)
    if (controllerOutput < 0)
    {
      // CO is negative
      leftSpeed = getWheel(controllerOutput, 1);
      rightSpeed = getWheel(controllerOutput, -1);
    }
    // too far right from target (go left)
    else
    {
      // CO is positive
      rightSpeed = getWheel(controllerOutput, -1);
      leftSpeed = getWheel(controllerOutput, 1);
    }

    // lower limit check
    if (leftSpeed < MOTOR_MIN_SPEED)
      leftSpeed = MOTOR_MIN_SPEED;
    if (rightSpeed < MOTOR_MIN_SPEED)
      rightSpeed = MOTOR_MIN_SPEED;
    // max limit check
    if (leftSpeed > MOTOR_MAX_SPEED)
      leftSpeed = MOTOR_MAX_SPEED;
    if (rightSpeed > MOTOR_MAX_SPEED)
      rightSpeed = MOTOR_MAX_SPEED;

    // motors are backwards
    motors.setSpeeds(rightSpeed, leftSpeed);

    mT2 = mT1;
  }
}

// calculates the wheelspeed
double getWheel(double co, int sign)
{
  // Multiple the magnitude by the MOTOR_FACTOR to map the magnitude range(0 - 100) at the motors
  //(0 - MOTOR_BASED_SPEED)
  // Added Min Speed as a constant to increase the speed
  double wheelSpeed = (MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR)) + (co * sign) + (MOTOR_MIN_SPEED);

  // Slows down when objects are in front of it
  if (DISTANCES[currHeadPos] < slowDist)
    wheelSpeed = wheelSpeed * (dist / slowDist);

  // slows the robot down when it is approaching the end goal
  // Wheel gradually slows down when the slow dist is reached
  if (dist < slowDist)
    wheelSpeed = wheelSpeed * (dist / slowDist);

  if (MOTOR_DEBUG)
    debugMotors("Motors");

  return wheelSpeed;
}

void debugLocation(char label[])
{
  Serial.print(label);
  Serial.print(Sl);
  Serial.print(",");
  Serial.print(Sr);

  Serial.print(",");
  Serial.print(tempSl);
  Serial.print(",");
  Serial.print(tempSr);
  Serial.print(",");
  Serial.print(travelDist);

  Serial.print(xGoal);
  Serial.print(",");
  Serial.print(yGoal);

  Serial.print(",");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.print(theta);
  Serial.print(",");
  Serial.println(dist);
}

void debugMotors(char label[])
{
  Serial.print(label);

  Serial.print(leftSpeed);
  Serial.print(",");
  Serial.print(rightSpeed);

  Serial.print(",");
  Serial.print((MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR)));
}

void debugUltrasonic(char label[])
{
  Serial.print(label);
  Serial.print(detectionLevel);
  Serial.print(",");
  Serial.print(usdetection);
  Serial.print(",");
  Serial.println(closeObj);
}

void debugPID(char label[])
{
  Serial.print(label);
  Serial.print(theta);
  Serial.print(",");
  Serial.println(gain);
}

void debugDistances(char label[])
{
  Serial.print(label);
  for (int i = 0; i < (sizeof(DISTANCES) / sizeof(long)); i++)
  {
    Serial.print(DISTANCES[i]);
    Serial.print(",");
  }
  Serial.println("");
}

void debugHeadServo(char label[])
{
  Serial.print(label);
  Serial.print(currHeadPos);
  Serial.print(",");
  Serial.println(sweepingClockwise);
}

void debugEncoders(char label[])
{
  Serial.print(label);
  Serial.print(",");
  Serial.print(sChange);
  Serial.print(",");
  Serial.print(thetaChange);
  Serial.print(",");
  Serial.print(xChange);
  Serial.print(",");
  Serial.println(yChange);
}