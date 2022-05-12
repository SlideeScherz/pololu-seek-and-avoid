/**
 * Name: pololu_seek_and_avoid.ino
 * Created: 5/9/2022 12:36:13 AM
 * Authors: Team Five
 *
 * All distance calculations done in CM.
 * All timer data will be in milliseconds
 * Time and delay for ulatrasonic sending and receiving pings will be MICROSECONDS. See Docs
 * Any reference to headServo will be for the sweeping servo the ultrasonic is attached to. Not a wheel motor.
 */

#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include "Coordinate.h"
#include "Clock.h"

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
Servo headServo;

// pin assignments
const uint8_t TRIG_PIN = 11;
const uint8_t ECHO_PIN = 5;
const uint8_t HEAD_SERVO_PIN = 4;

/**
 * debug switches
 * Data outputted in CSV format
 * Disable to boost performance
 * If not using any, set CSV_PERIOD to 10,000 ms
 */

const bool LOC_DEBUG = false;        // localization
const bool ENCODER_DEBUG = false;    // wheel encoders
const bool MOTOR_DEBUG = false;      // wheel motors
const bool PID_DEBUG = false;        // pid and erros
const bool REP_FORCES_DEBUG = false; // repulsive forces containers
const bool HEAD_SERVO_DEBUG = false; // head servo pos, angle
const bool US_DEBUG = false;         // ultrasonic

/* scheduler data */

Clock usTimer(15ul);
Clock motorTimer(20ul);
Clock encoderTimer(20ul);
Clock headServoTimer(50ul);
Clock csvTimer(50ul);

// misc constants
const double SPEED_OF_SOUND = 0.034;
constexpr int POS_LEN = 5; // default array length

/* head servo data */

bool sweepingClockwise = true; // sweeping right or left

// angles head servo can point
const int HEAD_POSITIONS[POS_LEN] = { 50, 70, 90, 110, 130 };

// index of arrays, init to 1, will cycle on start
int servoPosition = 1;

// angle servo is currently facing
int servoAngle = HEAD_POSITIONS[servoPosition];

/* us data */

// distance limits
const double MIN_DISTANCE = 2.0, MAX_DISTANCE = 80.0;

// time of round trip ping
unsigned long pingTimeDuration = 0ul;

// derived from duration
double pingDistance = 0.0;

// microsecond timeout (too far)
//const unsigned long PING_TIMEOUT = 38000ul;
const unsigned long PING_TIMEOUT = 5000ul;

// distance readings from each position
double distances[POS_LEN] = { MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE };

/* encoder data */

// encoder counts
long countsL = 0l, countsR = 0l;

// previous counts
long prevCountsL = 0l, prevCountsR = 0l;

// distance traveled by wheel in cm
double distL = 0.0, distR = 0.0;

// previous distance traveled
double prevDistL = 0.0, prevDistR = 0.0;

// difference between current and previous distance traveled
double deltaDistL = 0.0, deltaDistR = 0.0;

// change in distance traveled between last 2 intervals
double deltaDistTotal = 0.0;

/* wheel data */

// wheel and encoder constants, turtle edition
const double CLICKS_PER_ROTATION = 12.0;

 const double GEAR_RATIO = 75.81; // turtle edition
//const double GEAR_RATIO = 29.86; // standard edition
const double WHEEL_DIAMETER = 3.2;

// cm traveled each gear tick
const double DIST_PER_TICK = (WHEEL_DIAMETER * PI) / (CLICKS_PER_ROTATION * GEAR_RATIO);

// distance between the 2 drive wheels from the center point of the contact patches
const double B = 8.5;

/* position data */

// positional polar coordinates
Coordinate pos;

// change in position between last 2 intervals
Coordinate deltaPos;

/* goal data */

// goal status
bool xAccepted = false, yAccepted = false, goalComplete = false;

// x y coordinates of goal
Coordinate goal(50.0, 80.0);

// allow a slight error within this range
const double GOAL_PRECISION = 1.0;

/* motor data */

// motor speed limits
const int MIN_SPEED = 40, MAX_SPEED = 80;

// wheelSpeed containers. Set by PID + Repulsive forces
double speedL = 0.0, speedR = 0.0;

/* PID data */

const double KP = 27.5;

// suggested PID correction
double gain = 0.0;

// current theta vs theta of goal. Derived from arctan
double angleError = 0.0;

// current linear distance from goal. Updated on motor period
double distanceError = 0.0;

/* repulsive fields data */

// repellant factor for obstacles
const double R_FORCE_FACTORS[POS_LEN] = { 0.10, 0.50, 2, 0.50, 0.10 };

// container for all forces
double rForcesAll[POS_LEN] = { MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE };

// net repulsive force applied to plant
double rForceL = 0.0, rForceFwd = 0.0, rForceR = 0.0;

void setup()
{
  Serial.begin(9600);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  headServo.attach(HEAD_SERVO_PIN);

  // delete this block if not using robot backwards
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);
  encoders.flipEncoders(true);

  headServoTimer._ready = true;
  headServoTimer.taskPeriod = 50ul;

  usTimer._ready = true;

  motorTimer._ready = true;
  encoderTimer._ready = true;
  csvTimer._ready = true;

  // if 0 can damage breadboard
  if (servoAngle != 0)
    headServo.write(servoAngle);

  // init errors
  distanceError = eucDistance(pos, goal);
  angleError = getAngleError(pos, goal);

  delay(1000ul); // dont you run away from me...
  //buzzer.play("c32");

  printDebugHeadings();
}

void loop()
{
  if (!goalComplete)
  {
    setServo();
    readUltrasonic(servoPosition);

    // calls localize, pid, repulsive forces after
    readEncoders();
    setMotors();

    printDebugData();
  }

  // sleep when done
  else if (goalComplete)
  {
    //buzzer.play("c32");
    motors.setSpeeds(0, 0);
    while (true)
    {
      ledGreen(true);
      delay(1000);
      ledGreen(false);
    }
  }
}

// util methods

/**
 * @brief Calculate Euclidian distance
 * @param p1 position
 * @param p2 goal
 * @return double distance from target points
 */
double eucDistance(Coordinate p1, Coordinate p2)
{
  return sqrt(sq(p2.x - p1.x) + sq(p2.y - p1.y));
}

/**
 * @brief Get the Angle Error of robot vs target
 * @param p position
 * @param g goal
 * @return double error of orientation
 */
double getAngleError(Coordinate p, Coordinate g)
{
  return p.theta - atan2(g.y - p.y, g.x - p.x);
}

/**
 * @brief check max and min for an input value
 * @param input distance, speed, voltage, etc
 * @param min floor value
 * @param max ceiling value
 * @returns int modified value of bounds exceeded
 */
int handleLimit(int input, int min, int max)
{
  if (input <= min)
    input = min;
  else if (input >= max)
    input = max;

  return input;
}

/**
 * @brief check status of goals.
 * @param p position
 * @param g goal
 * @param errorThreshold value to adjust goal
 * @return true both accepted within acceptable goal
 * @return false not within acceptable goal
 */
bool checkGoalStatus(Coordinate p, Coordinate g, double errorThreshold)
{
  xAccepted = g.x - errorThreshold <= p.x && goal.x + errorThreshold >= p.x;
  yAccepted = g.y - errorThreshold <= p.y && goal.y + errorThreshold >= p.y;

  // check completed goal and set status
  return (xAccepted && yAccepted);
}

/**
 * @brief Get the Distance From TOF of ping
 * @param tof ping time duration
 * @param max value to return if timeout
 * @return double distance of ping
 */
double getDistanceFromTOF(unsigned long tof, double max)
{
  // handle timeout from US
  if (tof == 0ul)
    return max;

  else
    return tof * SPEED_OF_SOUND / 2.0;
}

/**
 * @brief Set the Servo object.
 * 1. Cycle position
 * 2. Write angle from head position array
 * 3. Move, wait for time to finish, reset switch
 */
void setServo()
{
  if (headServoTimer.ready() && !headServoTimer._running)
  {
    headServoTimer.start();
    usTimer._blocked = true;

    // get next position
    servoPosition = cyclePosition(servoPosition);
    servoAngle = HEAD_POSITIONS[servoPosition];

    headServo.write(servoAngle);
  }

  // allow servo to finish sweep
  else if (headServoTimer.completed())
  {
    headServoTimer.reset();
    usTimer._blocked = false;
  }
}

// cycle through HEAD_POSITIONS array
int cyclePosition(int posItr)
{
  // check bounds, toggle sweep
  if (posItr == 0 || posItr == POS_LEN - 1)
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
  // send one ping, write to correct index
  if (usTimer.ready() && !usTimer._blocked)
  {
    usTimer.start();

    pingDistance = sendPing();

    pingDistance = handleLimit(pingDistance, MIN_DISTANCE, MAX_DISTANCE);

    // write data
    distances[posItr] = pingDistance;

    usTimer.reset();
  }
}

/**
 * @brief send US ping to determine distance
 * @returns pingDistance total TOF
 */
double sendPing()
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

  pingTimeDuration = getDistanceFromTOF(pingTimeDuration, MAX_DISTANCE);

  return pingTimeDuration;
}

/**
 * @brief Read encoder data to calculate the distance traveled
 * @returns void sets encoder variables for localization calculation
 */
void readEncoders()
{
  if (encoderTimer.ready())
  {
    encoderTimer.start();

    // read current encoder count
    countsL += encoders.getCountsAndResetLeft();
    countsR += encoders.getCountsAndResetRight();

    // update the distance traveled by each wheel
    distL += (countsL - prevCountsL) * DIST_PER_TICK;
    distR += (countsR - prevCountsR) * DIST_PER_TICK;

    // get change of current and previous distance traveled
    deltaDistL = distL - prevDistL;
    deltaDistR = distR - prevDistR;

    // write previous encoder count
    prevCountsL = countsL;
    prevCountsR = countsR;

    // write previous distance traveled
    prevDistL = distL;
    prevDistR = distR;

    // send encoder data to calculate x,y,theta position
    localize(servoPosition);

    encoderTimer.reset();
  }
}

/**
 * @brief Update the robots current position using wheel encoder data.
 * After localized, will call PID, repulsiveForces and check goal status
 * @param posItr servo position to write to array
 * @returns void updates position
 */
void localize(int posItr)
{
  // change in distance traveled
  deltaDistTotal = (deltaDistL + deltaDistR) / 2.0;

  // change in orientation
  deltaPos.theta = (deltaDistR - deltaDistL) / B;

  // get polar coordinates of x and y
  deltaPos.x = deltaDistTotal * cos(pos.theta + deltaPos.theta / 2);
  deltaPos.y = deltaDistTotal * sin(pos.theta + deltaPos.theta / 2);

  // update coordinates
  pos.x += deltaPos.x;
  pos.y += deltaPos.y;
  pos.theta += deltaPos.theta;

  // send position data to PID controller to get a correction
  gain = getPID(pos, goal);

  getRepulsiveForces(posItr);

  goalComplete = checkGoalStatus(pos, goal, GOAL_PRECISION);
}

/**
 * @brief get a proportionate correction based on current theta vs goal
 * @param p position
 * @param g goal
 * @returns double proportional angle correction
 */
double getPID(Coordinate p, Coordinate g)
{
  // distance error magnitude
  distanceError = eucDistance(p, g);

  // error magnitude: current state - target state
  angleError = getAngleError(p, g);

  return KP * angleError;
}

/**
 * @brief Calculate all repulsive forces in the environment
 * @param posItr current head pos
 */
void getRepulsiveForces(int posItr)
{
  // ex 80 - 75 * 1 = 5 (very small, far away)
  // ex 80 - 5 * 1 = 75 (very close, apply large force)
  rForcesAll[posItr] = (MAX_DISTANCE - distances[posItr]) * R_FORCE_FACTORS[posItr];

  rForceL = rForcesAll[0] + rForcesAll[1];
  rForceFwd = rForcesAll[2];
  rForceR = rForcesAll[3] + rForcesAll[4];
}

/**
 * @brief set motor speeds using attractive and repulsive fields.
 * Always add to left, subtract from right to avoid zigzag when crossing y axis
 * 1. gain > 0, gain will be negative when adding (turn right).
 *  - left + (-30) decreasing, right - (-30) increasing
 * 2. gain < 0, gain will be positive when adding (turn left).
 *  - left + (+30) increasing, right - (+30) decreasing
 * @returns void. sets left and right global wheelspeeds.
 */
void setMotors()
{
  if (motorTimer.ready())
  {
    motorTimer.start();

    speedL = MIN_SPEED + gain + rForceL;
    speedR = MIN_SPEED - gain + rForceR;

    // handle fwd forces. Slow down
    speedL = speedL - rForceFwd;
    speedR = speedR - rForceFwd;

    speedL = handleLimit(speedL, MIN_SPEED, MAX_SPEED);
    speedR = handleLimit(speedR, MIN_SPEED, MAX_SPEED);

    // do not adjust regardless of fwd or backwards use
    motors.setSpeeds(speedL, speedR);

    motorTimer.reset();
  }
}

// headings for csv export
void printDebugHeadings()
{
  Serial.print("\n"); // nextline

  // localization
  if (LOC_DEBUG)
  {
    Serial.print("timestamp,");
    Serial.print("X,");
    Serial.print("Y,");
    Serial.print("theta,");
  }

  // pid
  if (PID_DEBUG)
  {
    Serial.print("dError,");
    Serial.print("aError,");
    Serial.print("gain, ");
  }

  // motors
  if (MOTOR_DEBUG)
  {
    Serial.print("speedL,");
    Serial.print("speedR,");
    Serial.print("t1,");
    Serial.print("t2,");
    Serial.print("ready,");
    Serial.print("running,");
    Serial.print("blocked,");
  }

  // encoders
  if (ENCODER_DEBUG)
  {
    Serial.print("countsL,");
    Serial.print("countsR,");
    Serial.print("prevCountsL,");
    Serial.print("prevCountsL,");
    Serial.print("distL,");
    Serial.print("distR,");
    Serial.print("prevDistL,");
    Serial.print("prevDistR,");
    Serial.print("deltaDistL,");
    Serial.print("deltaDistR,");
    Serial.print("deltaDist,");
    Serial.print("t1,");
    Serial.print("t2,");
    Serial.print("ready,");
    Serial.print("running,");
    Serial.print("blocked,");
  }

  // head servo
  if (HEAD_SERVO_DEBUG)
  {
    Serial.print("sweepCW,");
    Serial.print("sAngle,");
    Serial.print("sPos,");
    Serial.print("t1,");
    Serial.print("t2,");
    Serial.print("ready,");
    Serial.print("running,");
    Serial.print("blocked,");
  }

  // ultrasonic
  if (US_DEBUG)
  {
    Serial.print("pTimeDuration,");
    Serial.print("pDistance,");
    Serial.print("[0],");
    Serial.print("[1],");
    Serial.print("[2],");
    Serial.print("[3],");
    Serial.print("[4],");
    Serial.print("t1,");
    Serial.print("t2,");
    Serial.print("ready,");
    Serial.print("running,");
    Serial.print("blocked,");
  }

  // repulsive feilds
  if (REP_FORCES_DEBUG)
  {
    Serial.print("rForceL,");
    Serial.print("rForceFwd,");
    Serial.print("rForceR,");
    Serial.print("[0],");
    Serial.print("[1],");
    Serial.print("[2],");
    Serial.print("[3],");
    Serial.print("[4],");
  }

  Serial.println(__TIMESTAMP__);
}

// export csv data for plotting and tuning
void printDebugData()
{
  if (csvTimer.ready())
  {
    csvTimer.start();

    // current timestamp
    Serial.print(csvTimer.t1);
    Serial.print(",");

    // localization
    if (LOC_DEBUG)
    {
      Serial.print(pos.x);
      Serial.print(",");
      Serial.print(pos.y);
      Serial.print(",");
      Serial.print(pos.theta);
      Serial.print(",");
    }

    // pid
    if (PID_DEBUG)
    {
      Serial.print(distanceError);
      Serial.print(",");
      Serial.print(angleError);
      Serial.print(",");
      Serial.print(gain);
      Serial.print(",");
    }

    // motors
    if (MOTOR_DEBUG)
    {
      Serial.print(speedL);
      Serial.print(",");
      Serial.print(speedR);
      Serial.print(",");
      Serial.print(motorTimer.t1);
      Serial.print(",");
      Serial.print(motorTimer.t2);
      Serial.print(",");
      Serial.print(motorTimer._ready);
      Serial.print(",");
      Serial.print(motorTimer._running);
      Serial.print(",");
      Serial.print(motorTimer._blocked);
      Serial.print(",");
    }

    // encoders
    if (ENCODER_DEBUG)
    {
      Serial.print(countsL);
      Serial.print(",");
      Serial.print(countsR);
      Serial.print(",");
      Serial.print(prevCountsL);
      Serial.print(",");
      Serial.print(prevCountsR);
      Serial.print(",");
      Serial.print(distL);
      Serial.print(",");
      Serial.print(distR);
      Serial.print(",");
      Serial.print(prevDistL);
      Serial.print(",");
      Serial.print(prevDistR);
      Serial.print(",");
      Serial.print(deltaDistL);
      Serial.print(",");
      Serial.print(deltaDistR);
      Serial.print(",");
      Serial.print(deltaDistTotal);
      Serial.print(",");
      Serial.print(encoderTimer.t1);
      Serial.print(",");
      Serial.print(encoderTimer.t2);
      Serial.print(",");
      Serial.print(encoderTimer._ready);
      Serial.print(",");
      Serial.print(encoderTimer._running);
      Serial.print(",");
      Serial.print(encoderTimer._blocked);
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
      Serial.print(headServoTimer.t1);
      Serial.print(",");
      Serial.print(headServoTimer.t2);
      Serial.print(",");
      Serial.print(headServoTimer._ready);
      Serial.print(",");
      Serial.print(headServoTimer._running);
      Serial.print(",");
      Serial.print(headServoTimer._blocked);
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
      Serial.print(usTimer.t1);
      Serial.print(",");
      Serial.print(usTimer.t2);
      Serial.print(",");
      Serial.print(usTimer._ready);
      Serial.print(",");
      Serial.print(usTimer._running);
      Serial.print(",");
      Serial.print(usTimer._blocked);
      Serial.print(",");
    }

    // repulsive feilds
    if (REP_FORCES_DEBUG)
    {
      Serial.print(rForceL);
      Serial.print(",");
      Serial.print(rForceFwd);
      Serial.print(",");
      Serial.print(rForceR);
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
    }

    Serial.print("\n");

    csvTimer.reset();
  }
}