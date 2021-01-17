#include<Wire.h>
#include<Adafruit_BMP280.h>
#include<stdlib.h>
#include<string.h>
Adafruit_BMP280 bmp;  //I2C
float altitude, temp;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
float rpmX,rpmY,rpmZ;
int packet_count=0;
String altstr;
String tempstr;
String packstr;
String x1;
String x2;
String x3;
String finalprint;
String a("0000");
String d("   "); //time 
String b(",");
String c("S1");
void setup() {
  Serial.begin(115200);
  Wire.begin();//RPM Measurement
  bmp1();
  setupMPU();
 
}

void loop() {
    packet_count=packet_count+1;
    packstr=String(packet_count);
    finalprint += a + b + d + packstr + b + c + b + altstr + b + tempstr + b + x1 + x2 + x3;
    Serial.println(finalprint);
    delay(115200);
    BMPMeasurement();
    recordGyroRegisters();//RPM Measurement
    RPMMeasurement();
}

void bmp1(){
  if (!bmp.begin(0x76)) {
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}              
void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission();
}
void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rpmX=rotX/6;
  rotY = gyroY / 131.0;
  rpmY=rotY/6;
  rotZ = gyroZ / 131.0;
  rpmZ=rotZ/6;
}
void RPMMeasurement() {
  x1=String(rpmX);
  x2=String(rpmY);
  x3=String(rpmZ);
}
void BMPMeasurement()
{
    temp=bmp.readTemperature();
    tempstr=String(temp);
    altitude=bmp.readAltitude(1013.25);
    altstr=String(altitude);
   
}
