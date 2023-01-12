
#include "mpu6500.h"
#include <math.h>

// float xAng, yAng, zAng;

float xAng = 0;
float yAng = 0;
float zAng = 0;

/* Mpu6500 object */
bfs::Mpu6500 imu;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
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

//Corrected data values x + 0.44, y + 0.44, z - 0.78
void loop() {
  /* Check if data read */
  if (imu.Read()) {

    xAng = xAng + checkIfValid(imu.gyro_x_radps());
    Serial.print(xAng);
    Serial.print("\t");
    yAng = yAng + checkIfValid(imu.gyro_y_radps());
    Serial.print(yAng);
    Serial.print("\t");
    zAng = zAng + checkIfValid(imu.gyro_z_radps());
    Serial.print(zAng);
    Serial.print("\n");
  }
}
