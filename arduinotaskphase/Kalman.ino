#include<Wire.h>



// Kalman filter variables
double Q_angle = 0.001;
double Q_velocity = 0.003;
double R_measure = 0.03;
double angle = 0; // Reset the angle
double bias = 0; // Reset bias
double pitch=0; 
double P[2][2] = {{0, 0}, {0, 0}};
double K[2]; // Kalman gain
float AccX, AccY, AccZ, GyyX, GyyY, GyyZ;
float RateRoll, RatePitch, AngleRoll, AnglePitch, RateYaw;
void SensorConfig()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096 - 0.05;
  AccZ = (float)AccZLSB / 4096 - 0.13;

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyyXLSB = Wire.read() << 8 | Wire.read();
  int16_t GyyYLSB = Wire.read() << 8 | Wire.read();
  int16_t GyyZLSB = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyyXLSB / 32.5 + 0.55;
  RatePitch = (float)GyyYLSB / 32.5 - 0.32;
  RateYaw = (float)GyyZLSB / 32.5 + 0.65;
  double temp_roll = sqrt(pow(AccX, 2) + pow(AccZ, 2));
  double temp_pitch = sqrt((pow(AccY, 2) + pow(AccZ, 2)));
  AngleRoll = atan2(AccY, temp_roll) * (180 / 3.14);
  AnglePitch = atan2(AccX, temp_pitch) * (180 / 3.14);
}

// Function to update the Kalman filter
void Kalman(double newAngle, double newRate, double dt) {
  // Predict
  angle += dt * (newRate - bias);
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_velocity * dt;

  // Update
  double S = P[0][0] + R_measure; //estimates the error.
  K[0] = P[0][0] / S; //kalman gain factor
  K[1] = P[1][0] / S;

  double y = newAngle - angle; //angle difference
  angle += K[0] * y; //updating the angle values
  bias += K[1] * y;
  //erro covariance matrix
  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];
}
void KalmanPitch(float newPitch, float newRate, double dt)
{
  //
  pitch += dt*(newRate - bias);
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_velocity * dt;

  // Update
  double S = P[0][0] + R_measure; //estimates the error.
  K[0] = P[0][0] / S; //kalman gain factor
  K[1] = P[1][0] / S;

  double y = newPitch-pitch;//pitch difference 
  pitch += K[0] * y; //updating the angle values
  bias += K[1] * y;
  //erro covariance matrix
  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
}

void loop() {
  SensorConfig();
  double measurement_angle = AngleRoll;
  double measurement_rate = AnglePitch;
  double dt = 0.01; // Time step in seconds

double measurement_pitch=AnglePitch; 
 measurement_rate=AngleRoll;
 KalmanPitch(measurement_pitch,measurement_rate,dt);
  Kalman(measurement_angle, measurement_rate, dt);
  Serial.print("Kalman Angle is: ");
  Serial.print(angle); Serial.print(","); Serial.println(pitch); 

}
