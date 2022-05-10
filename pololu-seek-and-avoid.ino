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
const int TRIG_PIN = 5;
const int ECHO_PIN = 4;
const int HEAD_SERVO_PIN = 18;

// debug switches
const bool MOTOR_DEBUG = false;
const bool LOCATION_DEBUG = false;
const bool ENCODER_DEBUG = false;
const bool HEAD_SERVO_DEBUG = false;
const bool PID_DEBUG = false;
const bool US_DEBUG = false;

/* debugging data */
bool bLogCSV = true;

// scheduler intervals
const unsigned long MOTOR_PERIOD = 20ul;       // motor speed
const unsigned long ENCODER_PERIOD = 20ul;     // count encoders
const unsigned long US_PERIOD = 15ul;          // ultrasonic ping
const unsigned long HEAD_SERVO_PERIOD = 200ul; // sweep head
const unsigned long csvPERIOD = 50uL;

// scheduler timers
unsigned long encodersT1 = 0ul, encodersT2 = 0ul;
unsigned long csvT1 = 0ul, csvT2 = 0ul;
unsigned long motorT1 = 0ul, motorT2 = 0ul;

// misc constants
// TODO data type
const double SPEED_OF_SOUND = 0.034;

// index for parsing pos, delta, goal arrays
constexpr int X = 0, Y = 1, THETA = 2;

//============================================

/* encoder data */

// encoder counts
long countsLeftT1 = 0, countsRightT1 = 0;

// container to store the previous counts
long countsLeftT2 = 0, countsRightT2 = 0;

// distance traveled by wheel in cm
float sLeftT1 = 0.0f, sRightT1 = 0.0f;

// container to store the previous distance traveled
float sLeftT2 = 0.0f, sRightT2 = 0.0f;

// difference between current and previous distance traveled
float sLeftDelta = 0.0f, sRightDelta = 0.0f;

// change in distance traveled between last 2 intervals
float sDelta = 0.0f;

/* wheel data */

// wheel and encoder constants, turtle edition
const float CLICKS_PER_ROTATION = 12.0f;
const float GEAR_RATIO = 75.81f;
const float WHEEL_DIAMETER = 3.2f;

// cm traveled each gear tick
const float DIST_PER_TICK = (WHEEL_DIAMETER * PI) / (CLICKS_PER_ROTATION * GEAR_RATIO);

// distance between the 2 drive wheels from the center point of the contact patches
const float B = 8.5f;

/* position data */

// positional polar coordinates
float pos[3] = { 0.0f, 0.0f, 0.0f };

// change in position between last 2 intervals
float posDelta[3] = { 0.0f, 0.0f, 0.0f };

/* goal data */

// index of GOAL array to select which goal to navigate to
int currentGoal = 0;

// len of GOALS array, how many goals we want to navigate to
const int NUM_GOALS = 1;

// goal containers
float xGoals[NUM_GOALS] = { 100.0f };
float yGoals[NUM_GOALS] = { 80.0f };

// coordinates of goal
float goal[2] = { xGoals[currentGoal], yGoals[currentGoal] };

// allow a slight error within this range
const float GOAL_PRECISION = 0.75f;

// starting linear distance from goal. Updated on goal change
float startGoalDistance = eucDistance(goal[X], goal[Y], pos[X], pos[Y]);

// current linear distance from goal. Updated on motor period
float currentGoalDistance = startGoalDistance;

/* motor data */
// distance before applying dampening break force
const float DAMPEN_RANGE = 20.0f;

// speed limits
const float MOTOR_MIN_SPEED = 50.0f, MOTOR_MAX_SPEED = 150.0f;

// speed constants
const float MOTOR_BASE_SPEED = 100.0f;

// wheelSpeed containers. Set by PID output
float leftSpeed = MOTOR_MIN_SPEED, rightSpeed = MOTOR_MIN_SPEED;

/* PID data */
// proportional gain
const float KP = 20.0f;

// suggested PID correction
float PIDCorrection = 0.0f;

// current theta vs theta of goal. Derived from arctan
float currentError = 0.0f;

// used in calculating error
float arctanToGoal = 0.0f;

void setup()
{
  Serial.begin(9600);
  delay(3000);
  printCSVHeadings();
}

void loop()
{
  if (currentGoal < NUM_GOALS)
  {
    readEncoders();
    setMotors();

    if (bLogCSV)
      logCSV();
  }

  // sleep when done
  else
  {
    ledGreen(true);
    delay(1000);
    ledGreen(false);
  }
}

bool goalAccepted(double position, double goal, double errorThreshold)
{
  return (goal - errorThreshold <= position && goal + errorThreshold >= position);
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

    if (ENCODER_DEBUG)
      debugEncoders();

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
  sDelta = (sLeftDelta + sRightDelta) / 2.0f;
  posDelta[THETA] = (sRightDelta - sLeftDelta) / B;

  // get polar coordinates of x and y
  posDelta[X] = sDelta * cos(pos[THETA] + posDelta[THETA] / 2.0f);
  posDelta[Y] = sDelta * sin(pos[THETA] + posDelta[THETA] / 2.0f);

  // update coordinates
  pos[X] += posDelta[X];
  pos[Y] += posDelta[Y];
  pos[THETA] += posDelta[THETA];

  // update position after getting encoder data
  currentGoalDistance = eucDistance(goal[X], goal[Y], pos[X], pos[Y]);

  // send position data to PID controller to get a correction
  getPIDCorrection();
}

/**
 * get a proportionate correction based on current theta vs goal
 * a positive currentError will suggest a left turn
 * a negative currentError will suggest a right turn
 * @param none. Reads position data and goal to set theta error
 * @returns void set PIDcorrection to a proportional angle correction
 */
void getPIDCorrection()
{
  arctanToGoal = atan2(goal[Y] - pos[Y], goal[X] - pos[X]);

  currentError = pos[THETA] - arctanToGoal;

  PIDCorrection = KP * currentError;

  checkGoalStatus();
}

/**
 * check status of goals
 * triggered when goalCompleted is set
 * @returns void. selects next goal and resets startGoalDistance
 */
void checkGoalStatus()
{
  bool goalCompleted = false;
  bool xAccepted = goalAccepted(pos[X], goal[X], GOAL_PRECISION);
  bool yAccepted = goalAccepted(pos[Y], goal[Y], GOAL_PRECISION);

  // check completed goal and set status
  if (xAccepted && yAccepted)
    goalCompleted = true;

  // advance to next goal
  if (goalCompleted)
  {
    // uncomment if you want to be annoyed
    // buzzer.play("c32");

    // cycle next goal
    currentGoal++;
    goal[X] = xGoals[currentGoal];
    goal[Y] = yGoals[currentGoal];

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
 * set motor speeds with PID input
 * @returns void. sets left and right global wheelspeeds.
 */
void setMotors()
{
  motorT1 = millis();

  if (motorT1 > motorT2 + MOTOR_PERIOD)
  {

    leftSpeed = MOTOR_BASE_SPEED + PIDCorrection;
    rightSpeed = MOTOR_BASE_SPEED - PIDCorrection;

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

    // round wheelspeeds
    // leftSpeed = floor(leftSpeed);
    // rightSpeed = floor(rightSpeed);

    motors.setSpeeds(leftSpeed, rightSpeed);

    motorT2 = motorT1;
  }
}

// export encoder data
void debugEncoders()
{
  Serial.print("ENC ");
  Serial.print("countsT1: ");
  Serial.print(countsLeftT1);
  Serial.print(", ");
  Serial.print(countsRightT1);
  Serial.print(" countsT2: ");
  Serial.print(countsLeftT2);
  Serial.print(", ");
  Serial.print(countsRightT2);
  Serial.print(" sT1: ");
  Serial.print(sLeftT1);
  Serial.print(", ");
  Serial.print(sRightT1);
  Serial.print(" sT2: ");
  Serial.print(sLeftT2);
  Serial.print(", ");
  Serial.print(sRightT2);
  Serial.print(" sDeltas: ");
  Serial.print(sLeftDelta);
  Serial.print(", ");
  Serial.print(sRightDelta);
  Serial.print(" posDelta: ");
  Serial.println(sDelta);
}

// headings for csv export
void printCSVHeadings()
{
  Serial.println(); // nextline
  Serial.println(__TIMESTAMP__);

  Serial.print("time,");
  Serial.print("X,");
  Serial.print("Y,");
  Serial.print("Theta,");
  Serial.print("xGoal,");
  Serial.print("yGoal,");
  Serial.print("goalDist,");
  Serial.print("atan,");
  Serial.print("error,");
  Serial.print("PID,");
  Serial.print("leftSpeed,");
  Serial.println("rightSpeed");
}

// export csv data for plotting and tuning
void logCSV()
{

  csvT1 = millis();

  if (csvT1 > csvT2 + csvPERIOD)
  {
    Serial.print(millis());
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
    Serial.print(PIDCorrection);
    Serial.print(",");
    Serial.print(leftSpeed);
    Serial.print(",");
    Serial.println(rightSpeed);

    csvT2 = csvT1;
  }
}