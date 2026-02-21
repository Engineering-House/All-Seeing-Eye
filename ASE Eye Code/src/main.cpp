/*
Serial Command key:

moveMtr <mtr select> <position (0,180)>
mtr 0: topEyelid
mtr 1: bottomEyelid
mtr 2: topEye
mtr 3: bottomEye

pointEye <point 1>, <point 2>, <point 3>
select a point in 3D cartesion coordinates and move eye to that pos

dumpData <cmd> <time>
cmd = t: dumpData = true
cmd = f: dumpData = false
cmd = c: change sendRate to time
time: transfer speed in ms

sendData
sends data

Data packet format
<yaw>, <pitch>, <roll>, <error code>
*/

/*
Remove print time for stuff running on core 0
instead have it shove values every cycle in eyeAngle and then have main loop print

In core 0 code, make it so it loops 500 times to calibrate the sensor before starting main loop
so we don't check bool every loop

check outputs from moving eye code

tune motor positions
*/

#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h>
#include <Wire.h>

//Global for IMU
TaskHandle_t Task1;
float q[4] = {1.0, 0.0, 0.0, 0.0};
int eyeAngle[3], plateAngle[3] = { 0 }; //Angle of eye
//int plateAngle[3] = { 0 }; //Angle of plate


/**************************\
*                          *
*   Code runing on core 1  *
*                          *
\**************************/


/*********************\
*    Globals core 1   *
\*********************/

//Serial communication global variables
char serialIn[100]; //The last serial communication
int serialLen = 0; //The length of the last serial communication
bool dumpData = false; //If esp32 should dump it's data, i.e. don't wait to send data
long unsigned int lastSend = 0; //Time since last data transmission when dumping data
int sendRate = 50; //Wait time before sending data [ms]
int erno = 0; //Error number, changed if program runs into an error

//Motor global variables
//Servo pins
int servoPin[4] = {13, -1, -1, -1};

//Servo objects
Servo topEyelid;
Servo bottomEyelid;
Servo topEye;
Servo bottomEye;
Servo* motors[4] = {&topEyelid, &bottomEyelid, &topEye, &bottomEye};


/*******************************\
* Funcition declarations core 1 *
\*******************************/

void readSerial();
void sendData();
void readMPU(void * pvParameters);

/*******************************\
*          Setup core 1         *
\*******************************/

void setup() {  
  //Start the serial
  Serial.begin(115200);

  //Start readMPU task on core 0
  xTaskCreatePinnedToCore(
      readMPU, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task1,  /* Task handle. */
      0); /* Core where the task should run */


  //Setup onboard LED to debugging
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  Serial.printf("Setup core: %i\n", xPortGetCoreID());

  //Initilize the servos and set them to pos 0
  for(int i = 0; i < 4; i++){
    motors[i]->setPeriodHertz(50);
    motors[i]->attach(servoPin[i], 200, 5000); //default 1000us to 2000us
    motors[i]->write(0);
  }
}

/*******************************\
*           Loop core 1         *
\*******************************/

void loop() {
  digitalWrite(2, LOW);
  
  //If there is something in the serial
  if (Serial.available()){
    digitalWrite(2, HIGH);
    readSerial(); //Calls function that reads the serial and stores the value in global var serialIn

    if (!strncmp(serialIn, "moveMotr", 8)){
      int motor = int(serialIn[9]) - int('0'); //Find the motor from the string
      int position = atoi(serialIn + 10); //Find the position from the string

      Serial.printf("Moving motor %i to positiong %i\n", motor, position);
      motors[motor]->write(position); //Write to the motor
      
    } else if (!strncmp(serialIn, "pointEye", 8)){
      Serial.println("start");
      int index = 9;
      int point[3] = { 0 };

      for (int i = 0; i < 3; i++){
        char temp[100] = { 0 };
        int j = 0;

        while(!(serialIn[index] == ',') && !(serialIn[index] == '\0')){
          if(int(serialIn[index]) >= int('0') && int(serialIn[index]) <= int('9')){
            temp[j] = serialIn[index];
            //Serial.printf("vector %d, char %c\n", i, serialIn[index]);
            j++;
          }

          //Serial.println(serialIn[index]);
          index++;
        }

        index += 2;
        temp[j + 1] = '\0';
        point[i] = atoi(temp);
      }

      //Serial.println("Printing positions");
      Serial.println(point[0]);
      Serial.println(point[1]);
      Serial.println(point[2]);

      
      double length = sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2]);
      double unitVector[3] = {double(point[0])/length, double(point[1])/length, double(point[2])/length};

      double topEyePos = M_PI/2 - acos(unitVector[0]); //Angle of unit vecotr in Z X plane
      double bottomEyePos = M_PI/2 - acos(unitVector[1]); //Angel of unit vector in Z Y plane

      Serial.println(length);
      Serial.printf("vec1 %d, vec2 %d, vec3, %d", unitVector[0], unitVector[1], unitVector[2]);
      Serial.println(M_PI);
      Serial.println(acos(.5));
      //Serial.println(topEyePos);
      //Serial.println(bottomEyePos);


    } else if (!strncmp(serialIn, "dumpData", 8)){
      if (serialIn[9] == 't'){ //Start sending data
        Serial.println("Sending data continuously");
        dumpData = true;
      } else if (serialIn[9] == 'f'){ //Stop sending data
        Serial.println("Stopping dataflow");
        dumpData = false;
      } else if (serialIn[9] == 'c'){ //Change the wait time for dumpData
        Serial.println("New data sendRate: ");
        sendRate = atoi(serialIn + 11);
        Serial.print(sendRate);
      }

    } else if (!strncmp(serialIn, "sendData", 8)){ //If serial requests data, send data
      sendData();
    }
  }

  if(dumpData){
    digitalWrite(2, HIGH);
  } else{
    digitalWrite(2, LOW);
  }

  //If esp should dump data & enough time has elapsed, send data
  if(dumpData && millis() >= (lastSend + sendRate)){
    sendData();
    lastSend = millis();
  }
}

//Send relative data through serial
void sendData(){
  Serial.printf("data: %d ,%d ,%d , ", eyeAngle[0], eyeAngle[1], eyeAngle[2]);
  //Serial.printf("%d, %d, %d", plateAngle[0], plateAngle[1], plateAngle[2]);
  //Serial.print(" ");
  Serial.println(erno);
}


//Read the serial and put it in an array, returns pointer to the array
void readSerial(){
  //Reset the current array
  for (int i = 0; i < sizeof(serialIn)/sizeof(serialIn[0]); i++){
    serialIn[i] = 0;
  }

  int index = 0;
  int startTime = millis();

  while(Serial.available() && (millis() < startTime + 1000)){
    char incomingChar = Serial.read();

    //If the end of the command is reached, return the message
    if (incomingChar == '\n'){
      serialIn[index] = '\0';
      serialLen = index;
      return;

    } else{ //Add char onto the array of chars
      serialIn[index] = incomingChar;
    }

    index++;
    delay(10);
  }
}



/**************************\
*                          *
*   Code runing on core 0  *
*                          *
\**************************/

//Uses mahony filtering to stabalize values
void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
  float tmp;

  //These variables might work
  float Ki = 0.0;
  float Kp = 30.0;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)

//  tmp = ax * ax + ay * ay + az * az;
   tmp = 0.0; //IGNORE ACCELEROMETER

  // ignore accelerometer if false (tested OK, SJR)
  if (tmp > 0.0)
  {

    // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity in the body frame (factor of two divided out)
    vx = q[1] * q[3] - q[0] * q[2];
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is cross product between estimated and measured direction of gravity in body frame
    // (half the actual magnitude)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Compute and apply to gyro term the integral feedback, if enabled
    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;  // integral error scaled by Ki
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;  // apply integral feedback
      gy += iy;
      gz += iz;
    }

    // Apply proportional feedback to gyro term
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  // Integrate rate of change of quaternion, given by gyro term
  // rate of change = current orientation quaternion (qmult) gyro rate

  deltat = 0.5 * deltat;
  gx *= deltat;   // pre-multiply common factors
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];

  //add qmult*delta_t to current orientation
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}


void readMPU(void * pvParameters){

  /*********************\
  *    Globals core 0   *
  \*********************/

  int MPU_addr = 0x68;
  int cal_gyro = 1;  //set to zero to use gyro calibration offsets below.

  // vvvvvvvvvvvvvvvvvv  VERY VERY IMPORTANT vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  //These are the previously determined offsets and scale factors for accelerometer and gyro for
  // a particular example of an MPU-6050. They are not correct for other examples.
  //The AHRS will NOT work well or at all if these are not correct

  float A_cal[6] = {0.0, 0.0, 0.0, 1.000, 1.000, 1.000}; // 0..2 offset xyz, 3..5 scale xyz
  float G_off[3] = { 0., 0., 0.}; //raw offsets, determined for gyro at rest
  #define gscale ((250./32768.0)*(PI/180.0))  //gyro default 250 LSB per d/s -> rad/s

  // ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // Free parameters in the Mahony filter and fusion scheme,
  // Kp for proportional feedback, Ki for integral
  float Kp = 30.0;
  float Ki = 0.0;

  // Notes: with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
  // with MPU-6050, some instability observed at Kp=100, Kp=30 works well

  // char s[60]; //snprintf buffer, if needed for debug

  // globals for AHRS loop timing
  float yaw, pitch, roll; //Euler angle output

  int print_ms = 300;

  /*********************\
  *     Setup core 0    *
  \*********************/

  Wire.begin(21,22);
  Serial.printf("Starting mpu readings on core %i\n", xPortGetCoreID());

  // initialize sensor
  // defaults for gyro and accel sensitivity are 250 dps and +/- 2 g
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  /*********************\
  *     Loop core 0     *
  \*********************/

  while(1){
    static unsigned int i = 0; //loop counter
    static float deltat = 0;  //loop time in seconds
    static unsigned long now = 0, last = 0; //micros() timers
    static long gsum[3] = {0};
    //raw data
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t Tmp; //temperature

    //scaled data as vector
    float Axyz[3];
    float Gxyz[3];


    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(true);
    Wire.requestFrom(MPU_addr, 14); // request a total of 14 registers
    int t = Wire.read() << 8;
    ax = t | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    t = Wire.read() << 8;
    ay = t | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    t = Wire.read() << 8;
    az = t | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    t = Wire.read() << 8;
    Tmp = t | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    t = Wire.read() << 8;
    gx = t | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    t = Wire.read() << 8;
    gy = t | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    t = Wire.read() << 8;
    gz = t | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    // calibrate gyro upon startup. SENSOR MUST BE HELD STILL (a few seconds)
    i++;
    if (cal_gyro) {

      gsum[0] += gx; gsum[1] += gy; gsum[2] += gz;
      if (i == 500) {
        cal_gyro = 0;  //turn off calibration and print results

        for (char k = 0; k < 3; k++) G_off[k] = ((float) gsum[k]) / 500.0;

        Serial.print("G_Off: ");
        Serial.print(G_off[0]);
        Serial.print(", ");
        Serial.print(G_off[1]);
        Serial.print(", ");
        Serial.print(G_off[2]);
        Serial.println();
      }
    }

    // normal AHRS calculations

    else {
      Axyz[0] = (float) ax;
      Axyz[1] = (float) ay;
      Axyz[2] = (float) az;

      //apply offsets and scale factors from Magneto
      for (i = 0; i < 3; i++) Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

      Gxyz[0] = ((float) gx - G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
      Gxyz[1] = ((float) gy - G_off[1]) * gscale;
      Gxyz[2] = ((float) gz - G_off[2]) * gscale;

      //  snprintf(s,sizeof(s),"mpu raw %d,%d,%d,%d,%d,%d",ax,ay,az,gx,gy,gz);
      //  Serial.println(s);

      now = micros();
      deltat = (now - last) * 1.0e-6; //seconds since last update
      last = now;

      Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

      // Compute Tait-Bryan angles. Strictly valid only for approximately level movement
      
      // In this coordinate system, the positive z-axis is down toward Earth.
      // Yaw is the angle between Sensor x-axis and Earth magnetic North
      // (or true North if corrected for local declination, looking down on the sensor
      // positive yaw is counterclockwise, which is not conventional for NED navigation.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
      // arise from the definition of the homogeneous rotation matrix constructed
      // from quaternions. Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order which for this configuration is yaw,
      // pitch, and then roll.
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.
        
        // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
        // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock

      roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
      pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
      //conventional yaw increases clockwise from North. Not that the MPU-6050 knows where North is.
      yaw   = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
      // to degrees
      yaw   *= 180.0 / PI;
      if (yaw < 0) yaw += 360.0; //compass circle
      //ccrrect for local magnetic declination here
      pitch *= 180.0 / PI;
      roll *= 180.0 / PI;


      eyeAngle[0] = yaw;
      eyeAngle[1] = pitch;
      eyeAngle[2] = roll;
    }
  }
}

