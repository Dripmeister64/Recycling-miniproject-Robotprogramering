#include <Wire.h>           //Include wire lib to enable i2c commuications
#include <Zumo32U4.h>
#include <math.h>

//Dokumentation af Zumo lib https://pololu.github.io/zumo-32u4-arduino-library/
//Dokumentation på linjesensor lib https://www.pololu.com/docs/0J19/all

//Instantiating classes as objects
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

//defining variables
double CPR = 909.7;                    //How many encoder pulses pr wheel rotation
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
//This function will calibrate the line sensors to the surface of the platform
{
  //Write on the onboard screen of the zumo that it will calibrate in 1 second
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
  //run loop for how many times the loopCount var tells it
  for (int i = 0; i < loopCount; i++)
  {
    //for the first 1/4 of the loop and the last 1/4 it will make the robot turn left
    if (i > (loopCount / 4) && i <= (loopCount - (loopCount / 4)))
    {
      motors.setSpeeds(-200, 200);
    }
    //For the remainder that is the midle 1/2 of the loop it turns right
    else
    {
      motors.setSpeeds(200, -200);
    }
    //For every iteration it will call the calibrate function, så that it can find the lowest measured value and the highest, when the robot is turning ontop of the line on the platform
    lineSensors.calibrate();
  }
  //After calibrating stop the motors and clear the display
  motors.setSpeeds(0, 0);
  display.clear();
}

void followLine() {
  //This function will make the robot stay on the middle of the line, with the help of a pid controller

  //Set the position var to the value of the line sensors, it will read all of the sensors and if the line is in the middle of the robot the value is around 2000 and if it is to the left it is under 2000 and to the right over 2000. QTR_EMITTERS_ON, 1 configures the function to read a white line on black background, by default it expects to read a black line on white background
  position = lineSensors.readLine(lineSensorValues, QTR_EMITTERS_ON, 1);

  //Defining error, error difference, and the previous error to be used in the PID controller, as described in the book Elements of Robotics by Mordechai Ben-Ari and Francesco Mondada.
  lferror = 2000 - position;
  lferrorDiff = lferror - lfprevError;
  lfprevError = lferror;
  //The PID expression according to the book Elements of robotics, note we excluded the integral part, since it worked better without it, the gains are defined in the start of the code and has been tuned through testing
  lfspeedDiff = lfkp * lferror + lfkd * lferrorDiff;

  //This will make the robot go straight, and then adjust its heading according to the output of the PID controller, with positive output on one wheel and negative output on the other to make the robot turn
  leftSpeed = 200 - lfspeedDiff;
  rightSpeed = 200 + lfspeedDiff;

  //Constraining the value to max 200 as to not go faster than motorpower 200
  leftSpeed = constrain(leftSpeed, 0 , 200);
  rightSpeed = constrain(rightSpeed, 0 , 200);

  motors.setSpeeds(leftSpeed, rightSpeed);

}

void followLineUntilIntersection() {
  //This function will make the robot follow the line until it detects the intersection on the platform
  while (true) {
    //Every iteration update the linseSensors values
    lineSensors.readLine(lineSensorValues, QTR_EMITTERS_ON, 1);
    //If the left most AND the right most line sensor detects the line, make the robot go a bit backwards and stop, and break the loop
    if (lineSensorValues[0] < 120 && lineSensorValues[4] < 120) {
      motors.setSpeeds(-400, -400);
      delay(30);
      motors.setSpeeds(0, 0);
      break;
    }
    //If the sensors does not see the line call thje followLine funciton to continue to follow the line
    else {
      followLine();
    }
  }
}

void driveStraightUntilLine() {
  //This function will make the robot go forwards until either one of the middle three sensors detects the line
  while (true) {
    //Update the sensor values for every iteration
    lineSensors.readLine(lineSensorValues, QTR_EMITTERS_ON, 1);
    Serial.println(lineSensorValues[2]);
    //If one of the middle three sensors detects the line the robots goes back a bit and stops and breaks the loop.
    if (lineSensorValues[2] < 130  || lineSensorValues[1] < 130 || lineSensorValues[3] < 130) {
      motors.setSpeeds(-200, -200);
      delay(100);
      motors.setSpeeds(0, 0);
      break;
    }
    //If no line is detected, go straight
    else {
      motors.setSpeeds(maxSpeed, maxSpeed);
    }
  }
}

void driveStraightUntilLine2() {
  //Same function as above function driveStraightUntilLine except this function only checks the outer mostt line senosr on each side
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
  //This function calibrates the reading from the gyroScope by filtering out some of the uncertainty
  
  delay(100);
  //for loop runs for 1024 iterations
  for (int i = 0; i < 1024; i++)
  {
    //Every tim e a new reading is ready from the gyroscope, it will read the neew value
    while (!imu.gyroDataReady()) {}
    //Serial.println("Inde i cal");
    //Read gyro value
    imu.readGyro();
    //Add all 1024 reading together
    gyroOffsetZ += imu.g.z;
  }
  //Divede by number of runs, to get the deviation of the gyro, to get more accurate reading from the gyroscope
  gyroOffsetZ /= 1024;

}

void updateAngleGyro() {
  //This function will update the estimation of the robots angle or heading
  
  // Figure out how much time has passed since the last update.
  m = micros();
  int dt = m - lastUpdate;
  lastUpdate = m;

  //Read the values from the gyroscope
  imu.readGyro();

  // Calculate how much the angle has changed, in degrees, and
  // add it to our estimation of the current angle.  The gyro's
  // sensitivity is 0.07 dps per digit.
  //The expression of the anlge calculation has been retrieved from polulu example code
  angle += ((float)imu.g.z - gyroOffsetZ) * 70 * dt / 1000000000;
}

void gyroTurn(int d) {
  //This function will make the robot turn out to a specific heading provided by the variable passed to it when called, it also uses a PID controller

  //Reset variables every time the function is called so it nows to start over
  angle = 0;
  lastUpdate = 0;
  m = 0;
  gterror = d - angle;

  //save current program time
  long startTime = millis();
  
  //While loop runs for 1.5 seconds, and the robot will adjust its heading in that time, which we found adequete through testing
  while ((millis() - startTime) < 1500) {
    //Every iteration update what heading the robot has
    updateAngleGyro();

    //Once again define erros variables for the PID controller
    gterror = d - angle;
    gterrorSum += gterror;
    gterrorDiff = gterror - gtprevError;
    gtprevError = gterror;

    //PID expression that will tell tthe robot how fastt it should turn in what direction to reach the target angle of the value passed to the function
    gtspeedDiff = gtkp * gterror + gtki * gterrorSum + gtkd * gterrorDiff;

    gtspeedDiff = constrain(gtspeedDiff, -400, 400);

    motors.setSpeeds(-gtspeedDiff, gtspeedDiff);
  }
  //Once the robot has turned to the specific heading, stop the motors.
  motors.setSpeeds(0, 0);
  //display.clear();
}

void turnOntoLine() {
  //This function will make the robot turn right untill itt detects the line

  //Start turning for a small amount of time before starting the loop, to avoid it detecting the line imediattly
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
  //This function will determine if a can is close or far away from the robot

  //Reset vbaribles every time the function is called
  longCount = 0;
  shortCount = 0;
  lastLongCount = 0;
  lastShortCount = 0;

  lineSensors.emittersOn();

  display.clear();
  display.print(F("Wow"));

  while (true) {

    //Every iteration read the proximity sensor values and count how many returns from the left and right IR LED when it pulses   
    proxSensors.read();
    int leftSensor = proxSensors.countsFrontWithLeftLeds();
    int rightSensor = proxSensors.countsFrontWithRightLeds();
    //Take average value
    int sensorAvg = (leftSensor + rightSensor) / 2;


    //The following 2 if chains determines if the current reading corrosponds to the can being close to the robot or far away, we picked these vlaues through testing
    if ((sensorAvg >= 2) && (sensorAvg <= 3)) {
      //Serial.println("Langt væk");
      longCount += 1;
    }

    if ((sensorAvg >= 4) && (sensorAvg <= 7)) {
      //Serial.println("Tæt på");
      shortCount += 1;
    }

    //Now we check if the current longCount is equal to the prevouis, if that is the case that means that it has gotten af reading that meant the can was close, and we want to make sure where the can is even when it missreads the distance values, so we check the last 10 reading to make sure they all say the same so we can be sure where the can is
    //If they are equal start over by resetting the counts.
    if (longCount == lastLongCount) {
      longCount = 0;
      lastLongCount = 0;
    }
    //Same as if statement before, just for the short counts
    if (shortCount == lastShortCount) {
      shortCount = 0;
      lastShortCount = 0;
    }

    //If it suceeds to get 10 of the same measurement the robot is certain if the can is close or far  away from the robot, resets the counts, and for the case where the can is far away call the fucntion that make the robot deposit the farthest can
    if (longCount == loopCount) {
      lineSensors.emittersOff();
      display.clear();
      display.print(F("Far"));
      longCount = 0;
      lastLongCount = 0;
      depositForwards();
      break;
    }

    //The same as the last if statement, but when iots certain that the can is close it will call tthe function that deposits the closest can
    if (shortCount == loopCount) {
      lineSensors.emittersOff();
      display.clear();
      display.print(F("Close"));
      shortCount = 0;
      lastShortCount = 0;
      depositSide();
      break;
    }

    //Every iteration we set the current countt to the last count so next time we compare to the previous count
    lastLongCount = longCount;
    lastShortCount = shortCount;
    delay(15);
  }
}

void depositForwards() {
  //This function will make the robot deposit the can when it is placed the farthest away, by driving straight until it sees the edge line, then turn not completely towards start, go 33 cm, then turn towards the start line, again
  
  motors.setSpeeds(200, 200);
  delay(300);
  //Drives straight until until the driuveStraightUntilLine function detects a line
  driveStraightUntilLine();
  delay(100);
  //Turn -157 degrees
  gyroTurn(-157);
  delay(100);
  //go 33 cm
  goDist(33.0);
  delay(100);
  motors.setSpeeds(0, 0);
  //Turn -110 degrees
  gyroTurn(-110);
  delay(100);

}

void goDist(double x) {
  //This function will make the robot go forwards by x centimeters determined by the value passed to the function when called

  //Reset encodercounts, so it only calculates its distance with values from the encoders when it is only travelling forwards
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  //REsetting variables every time the function is called so it does not use values from the previous time the function was called
  error = 1;
  countsLeft = 0;
  countsRight = 0;
  distL = 0;
  distR = 0;
  avgDist = 0;

  while (true) {
    //If the value the robot must travel is negative it is handled in this if statement
    if (x < 0) {
      //Go backwards
      motors.setSpeeds(-maxSpeed, -maxSpeed);

      //Add current encoder countts to the total, and reset them again
      countsLeft += encoders.getCountsAndResetLeft();
      countsRight += encoders.getCountsAndResetRight();

      //Calculate how far each track has travlled
      distL = revDist / CPR * (countsLeft * (-1));
      distR = revDist / CPR * (countsRight * (-1));

      //Get the average distance travelled between the tracks
      avgDist = ((distL + distR) / 2.0);

      //Calculate the current error, but when going backwards the function must change the sign of x
      error = x * (-1) - avgDist;

      //If the error is witin 0.2 to both sides, then break the loop
      if (-0.2 < error < 0.2) {

        break;

      }
    }

    //This if statement is the same one as above but this time when x is positive we want to go forwards
    if (x > 0) {
      
      //Instead ofgoing backwards as in the last statement the robot now go forwards
      motors.setSpeeds(maxSpeed, maxSpeed);


      countsLeft += encoders.getCountsAndResetLeft();
      countsRight += encoders.getCountsAndResetRight();

      distL = revDist / CPR * countsLeft;
      distR = revDist / CPR * countsRight;

      avgDist = (distL + distR) / 2.0;

      //When the robot should go forwards it does not need to time x with -1
      error = x - avgDist;

      //And again when the error is within 0.2 of both sides it breaks the loop
      if (-0.2 < error < 0.2) {

        break;

      }
    }
  }
  //Stop the robot
  motors.setSpeeds(0, 0);
}

void depositSide() {
  //This function will make the robot travel parallel to the belt, and then at the end of the belt make the robot go on the belt, and push the can to the other deposit side, untill it detects the edge line, where it will turn back around and go to the start position.
  
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
  //The setup function will only run once, and as the first thing whenthe program starts

  //Begin Serial and wire comm
  Serial.begin(9600);
  Wire.begin();

  //Initialize all the sensors we use
  proxSensors.initThreeSensors();
  proxSensors.setBrightnessLevels( brightnessLevels, 7);

  lineSensors.initFiveSensors();
  imu.init();
  imu.enableDefault();
  imu.configureForBalancing();

  //Print to the onboard screen, press button a to calibrate the line sensor
  display.clear();
  display.print(F("Tryk A"));
  display.gotoXY(0, 1);
  display.print(F("LS Kal."));
  //Halts the program until the button has been pressed
  buttonA.waitForButton();
  //call the calibrate line sensor function
  calibrateLineSensors();

  //Print to onboard screen press button to calibrate gyroscope
  display.clear();
  display.print(F("Tryk A"));
  display.gotoXY(0, 1);
  display.print(F("Gyro Cal"));
  buttonA.waitForButton();
  display.clear();

  delay(1000);

  //Call calGyro function when the button has been pressed
  calGyro();

  //Start the void loop when the buttton has been pressed after all the calibrations has been made
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
  //Start from the starting position and go forwards towards the line until it sees the line, 
  driveStraightUntilLine2();
  //Turn onto the line so the robot is ready to follow the line
  turnOntoLine();
  //Now the robot follows the line until it hits the intersection where it will wait for ther cans to roll by
  followLineUntilIntersection();
  buzzer.play(">g32>>c32");
  //Determine iof the can is close or far away from the robot, ad inside this function the functions to deposit the cans properly is called, as described earlier.
  determineDistance();
  buzzer.play(">g32>>c32");
  delay(100);
}
