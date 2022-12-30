#include <Wire.h>
#include <Zumo32U4.h>
#include <math.h>

//Dokumentation af Zumo lib https://pololu.github.io/zumo-32u4-arduino-library/
//Dokumentation på linjesensor lib https://www.pololu.com/docs/0J19/all


Zumo32U4OLED display;
//Zumo32U4LCD display;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4Motors motors;
Zumo32U4IMU imu;
Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Encoders encoders;

#define NUM_SENSORS 5
int lineSensorValues[NUM_SENSORS];

double CPR = 909.7;                    //How many encoder pulses pr wheel rotation
double DPR = CPR / 360;               //Degrees pr pulse
double revDist = 2 * PI * (3.7858647 / 2); //Distance travelled by 1 rotation of wheel by calculating circumfrence of wheel
double countsLeft = 0;
double countsRight = 0;
double distL = 0;
double distR = 0;
double avgDist = 0;
double error = 0;

int maxSpeed = 200;
int leftSpeed = 0;
int rightSpeed = 0;

float gyroOffsetZ = 0;
float angle = 0;

int brightnessLevels[7] = {0.2, 0.3, 0.5, 1, 2, 4, 6};
int loopCount = 10;
int longCount = 0;
int shortCount = 0;
int lastLongCount = 0;
int lastShortCount = 0;

int m = 0;
int lastUpdate = 0;

char dist;

//PID related vars for gyro turn
float gterror = 0;
float gterrorSum = 0;
float gterrorDiff = 0;
float gtprevError = 0;
float gtkp = 7;
float gtki = 0.06;
float gtkd = 2;
const float targetAngle = 0;
int gtspeedDiff = 0;
/////////////////////////


//PID related vars for line follow
int position = 0;
int lferror = 0;
int lferrorSum = 0;
int lfprevError = 0;
int lferrorDiff = 0;
float lfkp = 0.4;
float lfkd = 0.7;
int lfspeedDiff = 0;
/////////////////


////////////////////////////////
////////////////////////////////
////////////////////////////////

void calibrateLineSensors()
{
  display.clear();
  display.print(F("Kal. om"));
  display.gotoXY(0, 1);
  display.print(F("1 sek."));



  //Venter 1 sekund med at gå i gang med at kalibrere så man kan fjerne sin hånd efter man har sat robotten igen med at kaliberer
  delay(1000);

  display.clear();
  display.print(F("-|-|-|-|-"));
  display.gotoXY(0, 1);
  display.print(F("|-|-|-|-|"));
  int loopCount = 100;
  for (uint16_t i = 0; i < loopCount; i++)
  {
    if (i > (loopCount / 4) && i <= (loopCount - (loopCount / 4)))
    {
      motors.setSpeeds(-200, 200);
    }
    else
    {
      motors.setSpeeds(200, -200);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
  display.clear();
}

void followLine() {

  position = lineSensors.readLine(lineSensorValues, QTR_EMITTERS_ON, 1);

  lferror = 2000 - position;
  lferrorDiff = lferror - lfprevError;
  lfprevError = lferror;
  lfspeedDiff = lfkp * lferror + lfkd * lferrorDiff;

  leftSpeed = 200 - lfspeedDiff;
  rightSpeed = 200 + lfspeedDiff;

  leftSpeed = constrain(leftSpeed, 0 , 200);
  rightSpeed = constrain(rightSpeed, 0 , 200);

  motors.setSpeeds(leftSpeed, rightSpeed);

}

void followLineUntilIntersection() {
  while (true) {
    lineSensors.readLine(lineSensorValues, QTR_EMITTERS_ON, 1);
    if (lineSensorValues[0] < 120 && lineSensorValues[4] < 120) {
      motors.setSpeeds(-400, -400);
      delay(30);
      motors.setSpeeds(0, 0);
      break;
    }
    else {
      followLine();
    }
  }
}

void driveStraightUntilLine() {
  while (true) {
    lineSensors.readLine(lineSensorValues, QTR_EMITTERS_ON, 1);
    Serial.println(lineSensorValues[2]);
    if (lineSensorValues[2] < 130  || lineSensorValues[1] < 130 || lineSensorValues[3] < 130) {
      motors.setSpeeds(-200, -200);
      delay(100);
      motors.setSpeeds(0, 0);
      break;
    }
    else {
      motors.setSpeeds(maxSpeed, maxSpeed);
    }
  }
}

void driveStraightUntilLine2() {
  while (true) {
    lineSensors.readLine(lineSensorValues, QTR_EMITTERS_ON, 1);
    if (lineSensorValues[0] < 100 || lineSensorValues[4] < 100) {
      motors.setSpeeds(200, 200);
      delay(200);
      motors.setSpeeds(0, 0);
      break;
    }
    else {
      motors.setSpeeds(maxSpeed, maxSpeed);
    }
  }
}

void calGyro() {
  delay(100);
  for (uint16_t i = 0; i < 1024; i++)
  {
    while (!imu.gyroDataReady()) {}
    //Serial.println("Inde i cal");
    imu.readGyro();
    gyroOffsetZ += imu.g.z;
  }
  gyroOffsetZ /= 1024;

}

void updateAngleGyro() {
  // Figure out how much time has passed since the last update.
  //lastUpdate = 0;
  m = micros();
  int dt = m - lastUpdate;
  lastUpdate = m;

  imu.readGyro();

  // Calculate how much the angle has changed, in degrees, and
  // add it to our estimation of the current angle.  The gyro's
  // sensitivity is 0.07 dps per digit.
  angle += ((float)imu.g.z - gyroOffsetZ) * 70 * dt / 1000000000;
}

void gyroTurn(int d) {
  angle = 0;
  lastUpdate = 0;
  m = 0;
  gterror = d - angle;
  long startTime = millis();
  while ((millis() - startTime) < 1500) {
    updateAngleGyro();
    
    gterror = d - angle;
    gterrorSum += gterror;
    gterrorDiff = gterror - gtprevError;
    gtprevError = gterror;

    gtspeedDiff = gtkp * gterror + gtki * gterrorSum + gtkd * gterrorDiff;

    gtspeedDiff = constrain(gtspeedDiff, -400, 400);

    motors.setSpeeds(-gtspeedDiff, gtspeedDiff);
  }
  motors.setSpeeds(0, 0);
  //display.clear();
}

void turnOntoLine() {
  motors.setSpeeds(maxSpeed, -maxSpeed);
  delay(150);
  while (true) {
    lineSensors.readLine(lineSensorValues, QTR_EMITTERS_ON, 1);
    if (lineSensorValues[2] < 100) {
      motors.setSpeeds(0, 0);
      break;
    }
    else {
      motors.setSpeeds(maxSpeed, -maxSpeed);
    }
  }
}

void determineDistance() {

  longCount = 0;
  shortCount = 0;
  lastLongCount = 0;
  lastShortCount = 0;

  lineSensors.emittersOn();

  display.clear();
  display.print(F("Wow"));

  while (true) {
    //lineSensors.emittersOn();
    //lineSensors.readLine(lineSensorValues, QTR_EMITTERS_ON, 1);
    proxSensors.read();
    int leftSensor = proxSensors.countsFrontWithLeftLeds();
    int rightSensor = proxSensors.countsFrontWithRightLeds();
    int sensorAvg = (leftSensor + rightSensor) / 2;

    if ((sensorAvg >= 2) && (sensorAvg <= 3)) {
      //Serial.println("Langt væk");
      longCount += 1;
    }

    if ((sensorAvg >= 4) && (sensorAvg <= 7)) {
      //Serial.println("Tæt på");
      shortCount += 1;
    }

    if (longCount == lastLongCount) {
      longCount = 0;
      lastLongCount = 0;
    }

    if (shortCount == lastShortCount) {
      shortCount = 0;
      lastShortCount = 0;
    }

    if (longCount == loopCount) {
      lineSensors.emittersOff();
      display.clear();
      display.print(F("Far"));
      longCount = 0;
      lastLongCount = 0;
      depositForwards();
      break;
    }

    if (shortCount == loopCount) {
      lineSensors.emittersOff();
      display.clear();
      display.print(F("Close"));
      shortCount = 0;
      lastShortCount = 0;
      depositSide();
      break;
    }

    lastLongCount = longCount;
    lastShortCount = shortCount;
    delay(15);
  }
}

void depositForwards() {
  motors.setSpeeds(200, 200);
  delay(300);
  driveStraightUntilLine();
  delay(100);
  gyroTurn(-157);
  delay(100);
  goDist(33.0);
  delay(100);
  motors.setSpeeds(0, 0);
  gyroTurn(-110);
  delay(100);

}

void goDist(double x) {

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  error = 1;
  countsLeft = 0;
  countsRight = 0;
  distL = 0;
  distR = 0;
  avgDist = 0;

  while (true) {

    if (x < 0) {

      motors.setSpeeds(-maxSpeed, -maxSpeed);

      countsLeft += encoders.getCountsAndResetLeft();
      countsRight += encoders.getCountsAndResetRight();

      distL = revDist / CPR * (countsLeft * (-1));
      distR = revDist / CPR * (countsRight * (-1));

      avgDist = ((distL + distR) / 2.0);

      error = x * (-1) - avgDist;

      if (-0.2 < error < 0.2) {

        break;

      }
    }

    if (x > 0) {

      motors.setSpeeds(maxSpeed, maxSpeed);


      countsLeft += encoders.getCountsAndResetLeft();
      countsRight += encoders.getCountsAndResetRight();

      distL = revDist / CPR * countsLeft;
      distR = revDist / CPR * countsRight;

      avgDist = (distL + distR) / 2.0;

      error = x - avgDist;

      if (-0.2 < error < 0.2) {

        break;

      }
    }
  }
  motors.setSpeeds(0, 0);
}

void depositSide() {
  gyroTurn(-90);
  //delay(100);
  goDist(23.0);
  //delay(100);
  gyroTurn(90);
  //delay(100);
  goDist(12.0);
  //delay(100);
  gyroTurn(90);
  //delay(100);
  driveStraightUntilLine();
  gyroTurn(180);
  //delay(100);
  goDist(40.0);
  //delay(100);
  gyroTurn(-90);
  //delay(100);
  goDist(25);
  //delay(100);
  gyroTurn(-90);


}


////////////////////////////////
////////////////////////////////
////////////////////////////////

void setup() {

  Serial.begin(9600);
  Wire.begin();

  proxSensors.initThreeSensors();
  proxSensors.setBrightnessLevels( brightnessLevels, 7);

  lineSensors.initFiveSensors();
  imu.init();
  imu.enableDefault();
  imu.configureForBalancing();

  display.clear();
  display.print(F("Tryk A"));
  display.gotoXY(0, 1);
  display.print(F("LS Kal."));
  buttonA.waitForButton();

  calibrateLineSensors();

  display.clear();
  display.print(F("Tryk A"));
  display.gotoXY(0, 1);
  display.print(F("Gyro Cal"));
  buttonA.waitForButton();
  display.clear();

  delay(1000);

  calGyro();

  display.clear();
  display.print(F("Tryk A"));
  display.gotoXY(0, 1);
  display.print(F("For Start"));
  buttonA.waitForButton();

  delay(1000);
}

////////////////////////////////
////////////////////////////////
////////////////////////////////

void loop() {
  // put your main code here, to run repeatedly:
  driveStraightUntilLine2();
  turnOntoLine();
  //gyroTurn('r');
  followLineUntilIntersection();
  buzzer.play(">g32>>c32");
  determineDistance();
  buzzer.play(">g32>>c32");
  delay(100);
}
