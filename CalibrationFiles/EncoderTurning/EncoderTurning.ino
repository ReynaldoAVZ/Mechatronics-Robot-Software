#include <DualTB9051FTGMotorShieldUnoMega.h>
#include <Encoder.h>

DualTB9051FTGMotorShieldUnoMega motorShield;
Encoder leftEnc(20, 21);
Encoder rightEnc(18, 19);

double encoderDistance = -10;  // cm
double encoderRadius = 1;
double encoderDegrees = 90;
double encoderRadians = encoderDegrees * PI / 180;

unsigned long t_ms = 0;
double theta1, omega1, omega1_des, omega1f = 0;
double theta1_old = 0;
double theta1_des;
double theta1_final;
double theta2, omega2, omega2_des, omega2f = 0;
double theta2_old = 0;
double theta2_des;
double theta2_final;
double t, t_old, t0, deltaT;
double encoderKp = 20;  //PID gains
double error1, erro2;
double alpha1 = .3;  //digital filter weight
double alpha2 = .3;
double encoderRunTime = 2;

// Encoder variables
double leftEncCount, rightEncCount;
int leftEncM1C = 0, rightEncM2C = 0;  //declare and initialize motor commands
double leftEncV1M, rightEncV2M;
double GearRatio = 100;        // gear ratio
int encoderCountsPerRev = 64;  // encoder counts per Rev
double rw = 3.5;               // wheel radius in cm
double wheelD = 27.94;         // distance between wheels in cm
float encoderResolution = (360 / (encoderCountsPerRev * GearRatio)) * (PI / 180);

bool encoderRadiusTurn = false;
bool encoderDriveStraight = true;
void setup() {

  Serial.begin(9600);  //need to use high baud rate when printing during feedback control
  Serial.println("Arduino Ready to Receive Control Parameters");
  motorShield.init();
  motorShield.enableDrivers();

  t0 = micros() / 1000000.;  // do this once before starting feedback control
  t_old = 0;                 // do this once before starting feedback control
  leftEnc.readAndReset();
  rightEnc.readAndReset();
}

void loop() {
  if (encoderDriveStraight) {
    t = micros() / 1000000.0 - t0;
    deltaT = t - t_old;

    leftEncCount = leftEnc.read();
    rightEncCount = rightEnc.read();

    theta1 = leftEncCount * encoderResolution;
    omega1 = (theta1 - theta1_old) / deltaT;
    omega1 = alpha1 * omega1 + (1 - alpha1) * omega1f;

    theta2 = rightEncCount * encoderResolution;
    omega2 = (theta2 - theta2_old) / deltaT;
    omega2 = alpha2 * omega2 + (1 - alpha2) * omega2f;

    theta1_final = encoderDistance / rw;
    omega1_des = theta1_final / encoderRunTime;
    theta1_des += omega1_des * deltaT;

    theta2_final = encoderDistance / rw;
    omega2_des = theta2_final / encoderRunTime;
    theta2_des += omega2_des * deltaT;

    if (abs(theta1_des) < abs(theta1_final) && abs(theta2_des) < abs(theta2_final)) {
      leftEncV1M = encoderKp * (theta1_des - theta1);
      rightEncV2M = encoderKp * (theta2_des - theta2);
      // Debugging output
      // Serial.print("Current theta1: ");
      // Serial.print(theta1);
      // Serial.print("\tDesired theta1_des: ");
      // Serial.print(theta1_des);
      // Serial.print("\tFinal theta1: ");
      // Serial.println(theta1_final);

      // Serial.print("Current theta2: ");
      // Serial.print(theta2);
      // Serial.print("\tDesired theta2_des: ");
      // Serial.print(theta2_des);
      // Serial.print("\tFinal theta2: ");
      // Serial.println(theta2_final);

      // Serial.print("Left Motor Speed (V1M before constrain): ");
      // Serial.println(leftEncV1M);
      // Serial.print("Right Motor Speed (V2M before constrain): ");
      // Serial.println(rightEncV2M);
    } else {
      leftEncV1M = 0;
      rightEncV2M = 0;
      theta1 = theta1_old = omega1 = theta2 = theta2_old = omega2 = 0;
      theta1_des = theta2_des = theta1_final = theta2_final = 0;
      leftEncV1M = rightEncV2M = leftEncM1C = rightEncM2C = 0;
      encoderRadiusTurn = false;
      encoderDriveStraight = false;
    }

    leftEncV1M = constrain(leftEncV1M, -10, 10);
    rightEncV2M = constrain(rightEncV2M, -10, 10);

    leftEncM1C = 400 * leftEncV1M / 10;
    rightEncM2C = 400 * rightEncV2M / 10;

    // Serial.print("Constrained Left Motor Command (M1C): ");
    // Serial.println(leftEncM1C);
    // Serial.print("Constrained Right Motor Command (M2C): ");
    // Serial.println(rightEncM2C);

    motorShield.setSpeeds(rightEncM2C, leftEncM1C);

    theta1_old = theta1;
    theta2_old = theta2;
    omega1f = omega1;
    omega2f = omega2;

    t_old = t;
  }


  if (encoderRadiusTurn) {
    // Time and sample time calculation
    t = (micros() / 1000000.0) - t0;
    deltaT = t - t_old;

    // Encoder readings and current angle, velocity calculations
    leftEncCount = leftEnc.read();
    rightEncCount = rightEnc.read();

    theta1 = leftEncCount * encoderResolution;
    omega1 = alpha1 * ((theta1 - theta1_old) / deltaT) + (1 - alpha1) * omega1f;

    theta2 = rightEncCount * encoderResolution;
    omega2 = alpha2 * ((theta2 - theta2_old) / deltaT) + (1 - alpha2) * omega2f;

    // Define desired trajectory for the radius turn
    theta1_final = (encoderRadians * encoderRadius) / rw;
    omega1_des = theta1_final / encoderRunTime;
    theta1_des += omega1_des * deltaT;

    theta2_final = (encoderRadians * (encoderRadius + wheelD)) / rw;
    omega2_des = theta2_final / encoderRunTime;
    theta2_des += omega2_des * deltaT;

    // Control law application
    if (theta1_des < theta1_final && theta2_des < theta2_final) {
      leftEncV1M = encoderKp * (theta1_des - theta1);
      rightEncV2M = encoderKp * (theta2_des - theta2);
    } else {
      // Stop motors and reset state if the target is reached
      leftEncV1M = rightEncV2M = 0;
      theta1 = theta1_old = omega1 = theta2 = theta2_old = omega2 = 0;
      theta1_des = theta2_des = theta1_final = theta2_final = 0;
      leftEncV1M = rightEncV2M = leftEncM1C = rightEncM2C = 0;
      encoderRadiusTurn = false;
      encoderDriveStraight = false;
    }

    // Motor command application
    leftEncV1M = constrain(leftEncV1M, -10, 10);
    rightEncV2M = constrain(rightEncV2M, -10, 10);
    leftEncM1C = 400 * leftEncV1M / 10;
    rightEncM2C = 400 * rightEncV2M / 10;
    motorShield.setSpeeds(rightEncM2C, leftEncM1C);

    // Update values for the next iteration
    theta1_old = theta1;
    theta2_old = theta2;
    omega1f = omega1;
    omega2f = omega2;
    t_old = t;
  }
}
