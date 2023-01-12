#define echoPin 2
#define trigPin 3
#define LED 4
#include "mpu6500.h"
#include <math.h>
#include <Servo.h>    //include the servo library
#define servoPin 4
Servo myservo;        // create servo object to control a servo
float steeringAngle;  // variable to store the servo position

#define enA 5   //EnableA command line - should be a PWM pin
#define enB 6   //EnableB command line - should be a PWM pin

//name the motor control pins - replace the CHANGEME with your pin number, digital pins do not need the 'D' prefix whereas analogue pins need the 'A' prefix
#define INa 14  //Channel A direction 
#define INb 15  //Channel A direction 
#define INc 16  //Channel B direction 
#define INd 17  //Channel B direction 

byte speedSetting = 0;  //initial speed = 0

long duration;
int distance;
int delayPeriod;

// float xAng, yAng, zAng;

float xAng = 0;
float yAng = 0;
float zAng = 0;

/* Mpu6500 object */
bfs::Mpu6500 imu;


void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LED, OUTPUT);

  myservo.attach(servoPin);  //attach our servo object to pin D4
  //the Servo library takes care of defining the PinMode declaration (libraries/Servo/src/avr/Servo.cpp line 240)

  //configure the motor control pins as outputs
  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);   

  //initialise serial communication
  Serial.begin(9600);
  Serial.println("Arduino Nano is Running"); //sanity check

  speedSetting = 255;
  motors(speedSetting, speedSetting); //make a call to the "motors" function and provide it with a value for each of the 2 motors - can be different for each motor - using same value here for expedience
  Serial.print("Motor Speeds: ");
  Serial.println(speedSetting); 
    while(!Serial) {}
  /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);
  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }
  findError();
}

void loop() {
  // Move 1
  goForwards();
  delay(1000);
  stopMotors();
  delay(500);
  // Move 2
  goAntiClockwise();
  stopMotors();
  delay(500);
  // Move 3
  goBackwards();
  stopMotors();
  delay(500);
  // Move 4
  goClockwise();
  stopMotors();
  delay(500);
  // Move 5
  goBackwards();
  stopMotors();
  exit(0);
}

// MPU-9520 functions
void findError()
{
  float GyroX, GyroY, GyroZ, GyroErrorX, GyroErrorY, GyroErrorZ;
  int c;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    // Serial.println(GyroErrorX);
    // Serial.println(GyroErrorY);
    // Serial.println(GyroErrorZ);
    c++;
  }
  //Divide the sum by 200 to get the error value
  // GyroErrorX = GyroErrorX / 200;
  // GyroErrorY = GyroErrorY / 200;
  // GyroErrorZ = GyroErrorZ / 200;
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);

}

int checkIfValid(float deltaAngle)
{
  if (deltaAngle > 0.01)
  {
    return (deltaAngle);
  }
  else if (deltaAngle < -0.01)
  {
    return (deltaAngle);
  }
  else
  {
    return 0;
  }
}

float convertRadToDeg(float Angle)
{
  return((180/M_PI) * Angle);
}

void checkAngle(float rotationAngle)
{
  if (imu.Read()) {
    do
    {
      xAng = xAng + checkIfValid(imu.gyro_x_radps());
      Serial.print(xAng);
    }
    while (xAng == rotationAngle);
    return 0;
  }
  else
  {
    Serial.print("Error in IMU connection");
    return 1;
  }    
}

// HC-SR04 functions
int distanceCheck()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.016;
  Serial.print(distance);
  Serial.println("cm");
  if (distance == 10)
  {
    return 0;
  }
}



void motors(int leftSpeed, int rightSpeed) {
  //set individual motor speed
  //direction is set separately

  analogWrite(enA, leftSpeed);
  analogWrite(enB, rightSpeed);
}

void moveSteering() {
  //you may need to change the maximum and minimum servo angle to have the largest steering motion
  int maxAngle = 90;
  int minAngle = 0;
  myservo.write(0);
  for (steeringAngle = minAngle; steeringAngle <= maxAngle; steeringAngle += 1) { //goes from minAngle to maxAngle (degrees)
    //in steps of 1 degree
    myservo.write(steeringAngle);   //tell servo to go to position in variable 'steeringAngle'
    delay(15);                      //waits 15ms for the servo to reach the position
  }
  for (steeringAngle = maxAngle; steeringAngle >= minAngle; steeringAngle -= 1) { // goes from maxAngle to minAngle (degrees)
    myservo.write(steeringAngle);   //tell servo to go to position in variable 'steeringAngle'
    delay(15);                      //waits 15 ms for the servo to reach the position
  }
}



// Used once
void goForwards() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

// Used twice
void goBackwards() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
  distanceCheck();
}

// Used once
void goClockwise() {
  float rotationAngle = 3 * M_PI / 2;
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
  checkAngle(rotationAngle);
}

// Used once
void goAntiClockwise() {
  float rotationAngle = M_PI;  
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
  checkAngle(rotationAngle);
}

// Used 5 times
void stopMotors() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}
