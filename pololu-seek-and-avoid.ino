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
const bool LOG_CSV = true;

// scheduler intervals
const unsigned long MOTOR_PERIOD = 20ul;       // motor speed
const unsigned long ENCODER_PERIOD = 20ul;     // count encoders
const unsigned long US_PERIOD = 15ul;          // ultrasonic ping
const unsigned long HEAD_SERVO_PERIOD = 200ul; // sweep head
const unsigned long CSV_PERIOD = 50uL;          // print csv row

// scheduler timers
unsigned long encodersT1 = 0ul, encodersT2 = 0ul;
unsigned long csvT1 = 0ul, csvT2 = 0ul;
unsigned long motorT1 = 0ul, motorT2 = 0ul;
unsigned long servoTimer1 = 0ul, servoTimer2 = 0ul;

// misc constants
// TODO data type
const double SPEED_OF_SOUND = 0.034;

// index for parsing pos, delta, goal arrays
constexpr int X = 0, Y = 1, THETA = 2;

/* encoder data */

// encoder counts
long countsLeftT1 = 0, countsRightT1 = 0;

// container to store the previous counts
long countsLeftT2 = 0, countsRightT2 = 0;

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
const double GEAR_RATIO = 75.81;
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

// goal containers
double xGoals[NUM_GOALS] = { 100.0 };
double yGoals[NUM_GOALS] = { 80.0 };

// coordinates of goal
double goal[2] = { xGoals[currentGoal], yGoals[currentGoal] };

// allow a slight error within this range
const double GOAL_PRECISION = 1;

// starting linear distance from goal. Updated on goal change
double startGoalDistance = eucDistance(goal[X], goal[Y], pos[X], pos[Y]);

// current linear distance from goal. Updated on motor period
double currentGoalDistance = startGoalDistance;

/* motor data */
// distance before applying dampening break force
const double DAMPEN_RANGE = 20.0;

// speed limits
const int MOTOR_MIN_SPEED = 50, MOTOR_MAX_SPEED = 150;

// speed constants
const double MOTOR_BASE_SPEED = 100.0;

// wheelSpeed containers. Set by PID output
double leftSpeed = MOTOR_MIN_SPEED, rightSpeed = MOTOR_MIN_SPEED;

/* PID data */
// proportional gain
const double KP = 20.0;

// suggested PID correction
double PIDCorrection = 0.0;

// current theta vs theta of goal. Derived from arctan
double currentError = 0.0;

// used in calculating error
double arctanToGoal = 0.0;

/* head servo data */

// angle servo is currently facing
int servoAngle = 90;

// index of HEAD_POSITIONS array 
int servoPosition = 3;

// logic for servo sweeping right or left
bool sweepingClockwise = true;

// logic to stop US from sending pings if US is moving
bool servoMoving = false;

// legal head positions (angles) servo can point
const int HEAD_POSITIONS[7] = { 135, 120, 105, 90, 75, 60, 45 };

// position readings from each angle
int distances[7] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL };

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

    if (LOG_CSV)
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

void setServo()
{
  servoTimer1 = millis();

  // poll servo
  if (servoTimer1 > servoTimer2 + HEAD_SERVO_PERIOD && !servoMoving)
  {
    servoMoving = true;

    // get next position
    servoPosition = cyclePosition(servoPosition);
    servoAngle = HEAD_POSITIONS[servoPosition];
    
    headServo.write(servoAngle);

    // reset timer
    servoTimer2 = servoTimer1;
  }

  // allow servo to finish sweep
  else if (servoTimer1 > servoTimer2 + HEAD_SERVO_PERIOD && servoMoving)
  {
    servoMoving = false;
    servoTimer2 = servoTimer1;

    //if (bDebugHeadServo) headServoDebug("Head Servo");
  }
}

int cyclePosition(int index)
{
  // check bounds, toggle sweep
  if (index == 0 || index == 6) sweepingClockwise = !sweepingClockwise;

  // CW: start at 0 then ascend
  if (sweepingClockwise) return (7 + index + 1) % 7;
  
  // CCW: start at 6 then decend
  else return (7 + index - 1) % 7;
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

/*
 * set the LEDS to on or off.
 * @param (color) false (off) or true (on)
 * @returns void. Sets the pololu LED pins
 */
void setLEDs(bool Y, bool G, bool R) {
  ledYellow(Y);
  ledGreen(G);
  ledRed(R);
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

// output data to serial monitor
void debugHeadServo(char label[])
{
  Serial.print(label);
  Serial.print(",");
  Serial.print(sweepingClockwise);
  Serial.print(",");
  Serial.print(servoAngle);
  Serial.print(",");
  Serial.print(servoPosition);
  Serial.print(servoTimer1);
  Serial.print(",");
  Serial.println(servoTimer2);
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

  if (csvT1 > csvT2 + CSV_PERIOD)
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