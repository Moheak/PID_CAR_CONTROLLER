#include <Wire.h> 



/////////////////////////////////////////////////
long loopTime = 10000;                                        // microseconds
unsigned long timer = 0;                                      // timer to excute code based on timming condition

unsigned long lastTime = 0,lastMilliPrint =0;                 // timer to excute code based on timming condition
int i=0;
//Compass axis
double compass_x_axis, compass_y_axis, compass_z_axis;       // compass raw (pure) data of magnatic fields in x, y, and z axis
float headingDegrees=6;                                      // heading angle or Yaw angle calculated essintially from gyrocompass with the help of gyroscope and accelerometer for angle correction
double acc_x, acc_y, acc_z;                                  //accelerometer axis variables

double gyro_pitch, gyro_roll, gyro_yaw;  // reading of raw gyro x , y ,z axis in (degree/secand)


float declination ;                       //Set the declination between the magnetic and geographic north.
float declination_degs,declination_mins;  //  

//Accelerometer and Gyro Variables

double gyro_roll_cal = 0.001, gyro_pitch_cal = 0.001, gyro_yaw_cal = 0.001; // initial calibration values of gyroscopes in deg/sec


float compass_x_horizontal, compass_y_horizontal;          // compass corrected data of magnatic fields in x and y axis

double roll_rad_acc, pitch_rad_acc;  // calculated roll angle & pitch angle (in rad) using acceleromter only
float  roll_deg_acc, pitch_deg_acc;  // calculated roll angle & pitch angle (in deg) using acceleromter only
float  thetaG, rollG;                // calculated pitch angle & roll angle (in deg) using gyroscope only
float  rollFold_acc, rollFnew_acc, pitchFold_acc, pitchFnew_acc; // filterd pitch angle & roll angle (in deg) using accelerometer only
float  compass_B[3] = {0.641753, -7.472186, -27.545260};                                                                      // calibration data of HMC8553L Compass
float  compass_A[3][3] = {{0.918762, -0.012436, -0.019393},{-0.012436, 0.955102,-0.004257},{-0.019393, -0.004257, 0.789150}}; // calibration data of HMC8553L Compass

int  cal_int=1000 ;                     // Gyro calibration Counter

float dt;
float pitch, roll;                      // final angels produced using both accelerometers and gyroscopes 

int compass_address = 0x1E;             // The I2C address of the HMC5883L is 0x1E in hexadecimal form.
int gyro_address = 105;                 // The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
int ADXL345_address = 0x53;             // The I2C address of the MS5611 barometer is 0x77 in hexadecimal form.

byte highByte, lowByte;                 // bytes to be sent to python if needed

unsigned long Millis1 = 0;              // Loop Timing 
unsigned long Millis2 = 0;              // Loop Timing
 

double analog_signal_0 = 0, analog_signal_1 = 0, analog_signal_2 = 0, recieved_Signal_0, recieved_Signal_1;

///////////////////////////////////////////////// PID VERIABLES ////////////////////////////////////////////
float KP=0.6,KI=0.001,KD=10,PIDval=0.0; //PID constant
long millsold,millsnew,det; //changing in time veriables 
float  targetangl=0,actualangle,angleerror=0,angleerrorold,deltaE;
float angleslope,angleErroreAreaOld=0,angleAreaNew;
long previousTime=0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define   m1 22
#define   m12 23
#define   m2 24
#define   m22 25
#define   en1  6
#define   en2  7
String msg,dangle="100",state="ON";
int pwm=50;
void DserialEvent(){
if (Serial1.available())
msg=Serial1.readStringUntil('\n');
dangle=msg.substring(0,2);
dangle=msg.substring(3,6);
//cutting message into pieces
}
void componant_setup(void){
//setting up motor controller pins
  pinMode(m1,OUTPUT);
  pinMode(m12,OUTPUT);
  pinMode(m2,OUTPUT);
  pinMode(m22,OUTPUT);
  pinMode(en1,OUTPUT);
  pinMode(en2,OUTPUT);

   Wire.begin();  
   Wire.setClock(1000);                        // enable I2C protocol 
   delay(500);                            // 
 // declination of magnatic north from geographic north: Tripoli Positive East 3 deg 22 minute
  declination_degs = 3;
  declination_mins=23;
  declination =  declination_degs + (1/60 * declination_mins);
  Serial1.begin(9600); //set up serial1
}
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
Serial1.begin(9600);
componant_setup();
 setup_compass();                       // Initiallize the compass and set the correct registers.
 gyro_setup();                          // activate gyroscope and accelerometer settings of GY80 IMU (Inertial Measurment Unit)
 read_compass();                        // reading compass values
 calibrate_gyro();                      // calibrate the gyro bias and remove it from its raw (real) signal
 Serial.println("good work"); 
 delay(200); 

}

void loop() {
  // put your main code here, to run repeatedly:

PIDval=PID(targetangl,KP,KD,KI);
ControleMotors(PIDval);
// monetiring values using chart 
// Serial.print(360);
// Serial.print(",");
// Serial.print(0);
// Serial.print(",");
// Serial.print(PIDval);
// Serial.print(",");
// Serial.print(headingDegrees);
// Serial.print(",");                                           // Serial.print(state);Serial.print(" state   dangle");
// Serial.println(dangle.toInt());

}












