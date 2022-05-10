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

/**
 * @brief Calculate Euclidian distance
 * @param x2 goal x
 * @param y2 goal y
 * @param x1 current x
 * @param y1 current y
 * @return double distance from target points
 */
double eucDistance(double x2, double y2, double x1, double y1)
{
  return sqrt(sq(x2 - x1) + sq(y2 - y1));
}

// pin assignments
const int TRIG_PIN = 17;
const int ECHO_PIN = 0;
const int HEAD_SERVO_PIN = 30;

// debug switches
const bool MOTOR_DEBUG = true;
const bool ENCODER_DEBUG = true;
const bool HEAD_SERVO_DEBUG = true;
const bool US_DEBUG = true;
const bool LOG_CSV = true;

// scheduler intervals
const unsigned long US_PERIOD = 15ul;               // ultrasonic ping
const unsigned long MOTOR_PERIOD = 20ul;            // motor speed
const unsigned long ENCODER_PERIOD = 20ul;          // count encoders
const unsigned long HEAD_SERVO_WAIT_PERIOD = 50ul;  // make US wait for move to finish
const unsigned long CSV_PERIOD = 50uL;              // print csv row
const unsigned long HEAD_SERVO_MOVE_PERIOD = 200ul; // call sweep head

// scheduler timers
unsigned long encodersT1 = 0ul, encodersT2 = 0ul;
unsigned long csvT1 = 0ul, csvT2 = 0ul;
unsigned long motorT1 = 0ul, motorT2 = 0ul;
unsigned long servoT1 = 0ul, servoT2 = 0ul;
unsigned long usT1 = 0ul, usT2 = 0ul;

// misc constants
const double SPEED_OF_SOUND = 0.034;

// index for parsing pos, delta, goal arrays
constexpr int X = 0, Y = 1, THETA = 2;

/* encoder data */

// encoder counts
long countsLeftT1 = 0l, countsRightT1 = 0l;

// container to store the previous counts
long countsLeftT2 = 0l, countsRightT2 = 0l;

// distance traveled by wheel in cm
double sLeftT1 = 0.0, sRightT1 = 0.0;

// container to store the previous distance traveled
double sLeftT2 = 0.0, sRightT2 = 0.0;

// difference between current and previous distance traveled
double sLeftDelta = 0.0, sRightDelta = 0.0;

// change in distance traveled between last 2 intervals
double sDelta = 0.0;

/* wheel data */

// wheel and encoder constants, turtle edition
const double CLICKS_PER_ROTATION = 12.0;

// TODO: FIX
// const double GEAR_RATIO = 75.81; //turtle edition
const double GEAR_RATIO = 29.86; // standard edition
const double WHEEL_DIAMETER = 3.2;

// cm traveled each gear tick
const double DIST_PER_TICK = (WHEEL_DIAMETER * PI) / (CLICKS_PER_ROTATION * GEAR_RATIO);

// distance between the 2 drive wheels from the center point of the contact patches
const double B = 8.5;

/* position data */

// positional polar coordinates
double pos[3] = { 0.0, 0.0, 0.0 };

// change in position between last 2 intervals
double posDelta[3] = { 0.0, 0.0, 0.0 };

/* goal data */

// index of GOAL array to select which goal to navigate to
int currentGoal = 0;

// len of GOALS array, how many goals we want to navigate to
const int NUM_GOALS = 1;

// coordinates of goal
double goal[2] = { 50.0, 50.0 };

// allow a slight error within this range
const double GOAL_PRECISION = 1.0;

// starting linear distance from goal. Updated on goal change
double startGoalDistance = eucDistance(goal[X], goal[Y], pos[X], pos[Y]);

// current linear distance from goal. Updated on motor period
double currentGoalDistance = startGoalDistance;

/* motor data */
// distance before applying dampening break force
const double DAMPEN_RANGE = 20.0;

// speed limits
// TODO FIX
// const int MOTOR_MIN_SPEED = 50, MOTOR_MAX_SPEED = 150; // turtle
const int MOTOR_MIN_SPEED = 20, MOTOR_MAX_SPEED = 80; // standard

// init as avg
const double MOTOR_BASE_SPEED = (MOTOR_MAX_SPEED + MOTOR_MIN_SPEED) / 2;

// wheelSpeed containers. Set by PID output
double leftSpeed = MOTOR_MIN_SPEED, rightSpeed = MOTOR_MIN_SPEED;

/* PID data */

const double KP = 20.0;

// suggested PID correction
double gain = 0.0;

// current theta vs theta of goal. Derived from arctan
double currentError = 0.0;

// used in calculating error
double arctanToGoal = 0.0;

/* head servo data */

// number of angles in HEAD_POSITIONS array
constexpr int POS_LEN = 5;
constexpr int POS_BEGIN = 0;
constexpr int POS_END = POS_LEN - 1;

// angle servo is currently facing
int servoAngle = 90;

// index of HEAD_POSITIONS array
int servoPosition = 2;

// logic for servo sweeping right or left
bool sweepingClockwise = true;

// logic to stop US from sending pings if US is moving
bool servoMoving = false;

// legal head positions (angles) servo can point
const int HEAD_POSITIONS[POS_LEN] = { 130, 110, 90, 70, 50 };

/* repulsive data */

// repellant factor for obstacles
const double POS_FACTOR[POS_LEN] = { 0.10, 0.20, 1, 0.20, 0.05 };

double rForceLeft = 0.0;
double rForceFwd = 0.0;
double rForceRight = 0.0;

// container for all forces
double rForcesAll[POS_LEN] = {};

/* us data */

const double US_MAX_DISTANCE = 80.0;

double pingTimeDuration = 0.0f;

double pingDistance = 0.0f;

double tempPingDistance = 0.0f;

// microsecond timeout
const unsigned long PING_TIMEOUT = 38000ul;

// position readings from each angle
double distances[POS_LEN] = { US_MAX_DISTANCE, US_MAX_DISTANCE, US_MAX_DISTANCE, US_MAX_DISTANCE, US_MAX_DISTANCE };

void setup()
{
  Serial.begin(9600);
  delay(3000);
  printDebugHeadings();
}

void loop()
{
  if (currentGoal < NUM_GOALS)
  {
    setServo();
    readUltrasonic(servoPosition);

    // calls localize, pid, repulsive forces after
    readEncoders();
    setMotors();

    if (LOG_CSV)
      printDebugData();
  }

  // sleep when done
  else
  {
    ledGreen(true);
    delay(1000);
    ledGreen(false);
  }
}

void setServo()
{
  servoT1 = millis();

  // poll servo
  if (servoT1 > servoT2 + HEAD_SERVO_MOVE_PERIOD && !servoMoving)
  {
    servoMoving = true;

    // get next position
    servoPosition = cyclePosition(servoPosition);
    servoAngle = HEAD_POSITIONS[servoPosition];

    headServo.write(servoAngle);

    // reset timer
    servoT2 = servoT1;
  }

  // allow servo to finish sweep
  else if (servoT1 > servoT2 + HEAD_SERVO_WAIT_PERIOD && servoMoving)
  {
    servoMoving = false;
    servoT2 = servoT1;
  }
}

// cycle through HEAD_POSITIONS array
int cyclePosition(int posItr)
{
  // check bounds, toggle sweep
  if (posItr == POS_BEGIN || posItr == POS_END)
    sweepingClockwise = !sweepingClockwise;

  // CW: start at 0 then ascend
  if (sweepingClockwise)
    return (POS_LEN + posItr + 1) % POS_LEN;

  // CCW: start at 6 then decend
  else
    return (POS_LEN + posItr - 1) % POS_LEN;
}

/**
 * @brief call US methods to calculate distance
 * @param posItr current index of head positions array
 */
void readUltrasonic(int posItr)
{
  usT1 = millis();

  // send one ping, write to correct index
  if (usT1 > usT2 + US_PERIOD && !servoMoving)
  {
    pingDistance = sendPing();

    // handle timeout
    if (pingDistance == 0)
      pingDistance = US_MAX_DISTANCE;

    // handle too far to care
    else if (pingDistance >= US_MAX_DISTANCE)
      pingDistance = US_MAX_DISTANCE;

    // ssave data
    distances[posItr] = pingDistance;

    usT2 = usT1;
  }
}

/**
 * send US ping to determine distance
 * @returns pingDistance total TOF
 */
float sendPing()
{
  // set trigger pin to low voltage
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // activate trigger pin for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  // clear trigger pin
  digitalWrite(TRIG_PIN, LOW);

  // read echo pin and read second wave travel time
  pingTimeDuration = pulseIn(ECHO_PIN, HIGH, PING_TIMEOUT);

  // handle timeout, skip computation
  if (pingTimeDuration == 0)
    tempPingDistance = 0.0f;

  // calculate round trip time of flight
  else
    tempPingDistance = pingTimeDuration * SPEED_OF_SOUND / 2.0;

  return tempPingDistance;
}

/**
 * Read encoder data to calculate the distance traveled
 * Also, calculate the change in position to calculate position
 * @returns void sets sL and sR delta for position calculation
 */
void readEncoders()
{
  encodersT1 = millis();

  if (encodersT1 > encodersT2 + ENCODER_PERIOD)
  {

    // read current encoder count
    countsLeftT1 += encoders.getCountsAndResetLeft();
    countsRightT1 += encoders.getCountsAndResetRight();

    // update the distance traveled by each wheel
    sLeftT1 += (countsLeftT1 - countsLeftT2) * DIST_PER_TICK;
    sRightT1 += (countsRightT1 - countsRightT2) * DIST_PER_TICK;

    // get change of current and previous distance traveled
    sLeftDelta = sLeftT1 - sLeftT2;
    sRightDelta = sRightT1 - sRightT2;

    // write previous encoder count
    countsLeftT2 = countsLeftT1;
    countsRightT2 = countsRightT1;

    // write previous distance traveled
    sLeftT2 = sLeftT1;
    sRightT2 = sRightT1;

    // reset timer
    encodersT2 = encodersT1;

    // send encoder data to calculate x,y,theta position
    localize();
  }
}

/**
 * Update the robots current position using wheel encoder data
 * @param none reads change from sL and sR
 * @returns void updates x,y, theta and currentGoalDistance
 */
void localize()
{
  // update position using the deltas of each
  sDelta = (sLeftDelta + sRightDelta) / 2.0;
  posDelta[THETA] = (sRightDelta - sLeftDelta) / B;

  // get polar coordinates of x and y
  posDelta[X] = sDelta * cos(pos[THETA] + posDelta[THETA] / 2);
  posDelta[Y] = sDelta * sin(pos[THETA] + posDelta[THETA] / 2);

  // update coordinates
  pos[X] += posDelta[X];
  pos[Y] += posDelta[Y];
  pos[THETA] += posDelta[THETA];

  // update position after getting encoder data
  currentGoalDistance = eucDistance(goal[X], goal[Y], pos[X], pos[Y]);

  // send position data to PID controller to get a correction
  gain = getPID(pos[THETA]);

  getRepulsiveForces(servoPosition);

  checkGoalStatus();
}

/**
 * get a proportionate correction based on current theta vs goal
 * a positive currentError will suggest a left turn
 * a negative currentError will suggest a right turn
 * @returns void set PIDcorrection to a proportional angle correction
 */
double getPID(double currentTheta)
{
  arctanToGoal = atan2(goal[Y] - pos[Y], goal[X] - pos[X]);

  currentError = currentTheta - arctanToGoal;

  return KP * currentError;
}

/**
 * @brief Calculate all repulsive forces in the environment
 * @param posItr current head pos
 */
void getRepulsiveForces(int posItr)
{
  rForcesAll[posItr] = distances[posItr] * POS_FACTOR[posItr];

  rForceLeft = rForcesAll[0] + rForcesAll[1];
  rForceFwd = rForcesAll[2];
  rForceRight = rForcesAll[3] + rForcesAll[4];
}

// see if robot is within accepted goal distance
bool goalAccepted(double position, double goal, double errorThreshold)
{
  return (goal - errorThreshold <= position && goal + errorThreshold >= position);
}

/**
 * check status of goals
 * triggered when goalCompleted is set
 * @returns void. selects next goal and resets startGoalDistance
 */
void checkGoalStatus()
{
  bool xAccepted = goalAccepted(pos[X], goal[X], GOAL_PRECISION);
  bool yAccepted = goalAccepted(pos[Y], goal[Y], GOAL_PRECISION);

  // check completed goal and set status
  if (xAccepted && yAccepted)
  {

    buzzer.play("c32");

    // cycle next goal
    currentGoal++;

    // update start goal distance
    startGoalDistance = eucDistance(goal[X], goal[Y], pos[X], pos[Y]);

    // sleep after returning home
    if (currentGoal == NUM_GOALS)
    {
      motors.setSpeeds(0, 0);
      ledGreen(1);
    }
  }
}

/**
 * set motor speeds using attractive and repulsive fields
 * @returns void. sets left and right global wheelspeeds.
 */
void setMotors()
{
  motorT1 = millis();

  if (motorT1 > motorT2 + MOTOR_PERIOD)
  {

    // handle left and right forces
    leftSpeed = MOTOR_BASE_SPEED + gain + rForceLeft;
    rightSpeed = MOTOR_BASE_SPEED - gain + rForceRight;

    // handle fwd forces. Slow down
    leftSpeed = leftSpeed - rForceFwd;
    rightSpeed = rightSpeed - rForceFwd;

    // reduce wheelspeed if within threshold
    if (currentGoalDistance <= DAMPEN_RANGE)
    {
      leftSpeed *= (currentGoalDistance / DAMPEN_RANGE);
      rightSpeed *= (currentGoalDistance / DAMPEN_RANGE);
    }

    // check max and min speed limits
    if (leftSpeed <= MOTOR_MIN_SPEED)
      leftSpeed = MOTOR_MIN_SPEED;
    else if (leftSpeed >= MOTOR_MAX_SPEED)
      leftSpeed = MOTOR_MAX_SPEED;

    if (rightSpeed <= MOTOR_MIN_SPEED)
      rightSpeed = MOTOR_MIN_SPEED;
    else if (rightSpeed >= MOTOR_MAX_SPEED)
      rightSpeed = MOTOR_MAX_SPEED;

    motors.setSpeeds((int)leftSpeed, (int)rightSpeed);

    motorT2 = motorT1;
  }
}

/*
 * set the LEDS to on or off.
 * @param (color) false (off) or true (on)
 * @returns void. Sets the pololu LED pins
 */
void setLEDs(bool Y, bool G, bool R)
{
  ledYellow(Y);
  ledGreen(G);
  ledRed(R);
}

// headings for csv export
void printDebugHeadings()
{
  Serial.println(); // nextline
  Serial.println(__TIMESTAMP__);

  // position
  Serial.print("time,");
  Serial.print("X,");
  Serial.print("Y,");
  Serial.print("T,");
  Serial.print("xGoal,");
  Serial.print("yGoal,");
  Serial.print("goalDist,");
  Serial.print("atan,");
  Serial.print("error,");
  Serial.print("gain,");

  // wheel speeds
  Serial.print("leftSpeed,");
  Serial.print("rightSpeed,");

  // encoders
  if (ENCODER_DEBUG)
  {
    Serial.print("countsLeftT1,");
    Serial.print("countsRightT1,");
    Serial.print("countsLeftT2,");
    Serial.print("countsRightT2,");
    Serial.print("sLeftT1,");
    Serial.print("sRightT1,");
    Serial.print("sLeftT2,");
    Serial.print("sRightT2,");
    Serial.print("sLeftDelta,");
    Serial.print("sRightDelta,");
    Serial.print("sDelta,");
  }

  // head servo
  if (HEAD_SERVO_DEBUG)
  {
    Serial.print("sweepingClockwise,");
    Serial.print("servoAngle,");
    Serial.print("servoPosition,");
    Serial.print("servoT1,");
    Serial.print("servoT2,");
  }

  // ultrasonic
  if (US_DEBUG)
  {
    Serial.print("pingTimeDuration,");
    Serial.print("pingDistance,");
    Serial.print("d0,");
    Serial.print("d1,");
    Serial.print("d2,");
    Serial.print("d3,");
    Serial.print("d4,");
    Serial.print("usT1,");
    Serial.print("usT2,");
  }

  // repulsive feilds
  Serial.print("rForceLeft,");
  Serial.print("rForceFwd,");
  Serial.print("rForceRight,");
  Serial.print("d0,");
  Serial.print("d1,");
  Serial.print("d2,");
  Serial.print("d3,");
  Serial.print("d4,");

  Serial.println("");
}

// export csv data for plotting and tuning
void printDebugData()
{

  csvT1 = millis();

  if (csvT1 > csvT2 + CSV_PERIOD)
  {
    // current timestamp
    Serial.print(csvT1);
    Serial.print(",");
    Serial.print(pos[X]);
    Serial.print(",");
    Serial.print(pos[Y]);
    Serial.print(",");
    Serial.print(pos[THETA]);
    Serial.print(",");
    Serial.print(goal[X]);
    Serial.print(",");
    Serial.print(goal[Y]);
    Serial.print(",");
    Serial.print(currentGoalDistance);
    Serial.print(",");
    Serial.print(arctanToGoal);
    Serial.print(",");
    Serial.print(currentError);
    Serial.print(",");
    Serial.print(gain);
    Serial.print(",");

    // wheel speeds
    Serial.print(leftSpeed);
    Serial.print(",");
    Serial.print(rightSpeed);
    Serial.print(",");

    // encoders
    if (ENCODER_DEBUG)
    {
      Serial.print(countsLeftT1);
      Serial.print(",");
      Serial.print(countsRightT1);
      Serial.print(",");
      Serial.print(countsLeftT2);
      Serial.print(",");
      Serial.print(countsRightT2);
      Serial.print(",");
      Serial.print(sLeftT1);
      Serial.print(",");
      Serial.print(sRightT1);
      Serial.print(",");
      Serial.print(sLeftT2);
      Serial.print(",");
      Serial.print(sRightT2);
      Serial.print(",");
      Serial.print(sLeftDelta);
      Serial.print(",");
      Serial.print(sRightDelta);
      Serial.print(",");
      Serial.print(sDelta);
      Serial.print(",");
    }

    // head servo
    if (HEAD_SERVO_DEBUG)
    {
      Serial.print(sweepingClockwise);
      Serial.print(",");
      Serial.print(servoAngle);
      Serial.print(",");
      Serial.print(servoPosition);
      Serial.print(",");
      Serial.print(servoT1);
      Serial.print(",");
      Serial.print(servoT2);
      Serial.print(",");
    }

    // ultrasonic
    if (US_DEBUG)
    {
      Serial.print(pingTimeDuration);
      Serial.print(",");
      Serial.print(pingDistance);
      Serial.print(",");
      Serial.print(distances[0]);
      Serial.print(",");
      Serial.print(distances[1]);
      Serial.print(",");
      Serial.print(distances[2]);
      Serial.print(",");
      Serial.print(distances[3]);
      Serial.print(",");
      Serial.print(distances[4]);
      Serial.print(",");
      Serial.print(usT1);
      Serial.print(",");
      Serial.print(usT2);
      Serial.print(",");
    }

    // repulsive feilds
    Serial.print(rForceLeft);
    Serial.print(",");
    Serial.print(rForceFwd);
    Serial.print(",");
    Serial.print(rForceRight);
    Serial.print(",");
    Serial.print(rForcesAll[0]);
    Serial.print(",");
    Serial.print(rForcesAll[1]);
    Serial.print(",");
    Serial.print(rForcesAll[2]);
    Serial.print(",");
    Serial.print(rForcesAll[3]);
    Serial.print(",");
    Serial.print(rForcesAll[4]);
    Serial.print(",");

    Serial.println("");

    csvT2 = csvT1;
  }
}