#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_MotorShield.h>
//#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "Wire.h"


#define SPEED_OFFSET 20 // motors deadzone
#define N_MEAS 100 // number of measurements for the accel/gyro offsets

#define PWM_FREQ 100 // frequency of motorshield, has to be low to prevent switching noise

#define G_from_MPU 16383.0 // this is for fullscale of +-2g, we have 15 bits of value which is 32767 max, half that is 16383
#define deg_from_MPU 131.1 // fullscale of +-250, 15 bits of value, 
#define deg_per_lsb 0.007629f 

#define LED_PIN 13


Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *m1= AFMS.getMotor(1);
Adafruit_DCMotor *m2 = AFMS.getMotor(2);


MPU6050 accelgyro;

int go = 1; // start/stop signal

// store accel/gyro values
int16_t ax, ay, az;  
int16_t gx, gy, gz;

// accel/gyro offsets
int16_t ax_off, ay_off, az_off;
int16_t gx_off, gy_off, gz_off; 

float x,y,z, xx,yy,zz; // x y z, rx ry rz

int16_t ang; // angle from the horizontal plane

float ang_f; // angle in float
float acc_ang;  // angle from the accelerometer
float gy_f; // ang vel from gyroscope
float dt_f;


// PID parameters
float P = 28;
float I = 0.01;
float D = 18;

// for PID
float err;
float prev_err;
float d_err;
float cum_err;


float targ_ang = -90; // stability angle
float cmd;

unsigned long last_us;
unsigned long us;
unsigned long dt;

unsigned long last_ms;
unsigned long last_speed_ms;


bool blinkState = false;

void setup() {
    Wire.begin();
    Serial.begin(115200);

//    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_188); 
//    Serial.println("Testing device connections...");
//    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // initialize
    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);
 

    pinMode(LED_PIN, OUTPUT);


    // compute offsets
    long int a0_off = 0; 
    long int a1_off = 0; 
    long int a2_off = 0; 
    long int g0_off = 0; 
    long int g1_off = 0;
    long int g2_off = 0;

    // large sum
    for(int i = 0; i < N_MEAS; i++)
    {
      delay(50);
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      a0_off += ax;
      a1_off += ay;
      a2_off += az;
      g0_off += gx;
      g1_off += gy;
      g2_off += gz;
    }

    // average
    a0_off /= N_MEAS;
    a1_off /= N_MEAS;
    a2_off /= N_MEAS;
    g0_off /= N_MEAS;
    g1_off /= N_MEAS;
    g2_off /= N_MEAS;

    // write to globals
    ax_off = a0_off;
    ay_off = a1_off;
    az_off = a2_off + G_from_MPU;
    gx_off = g0_off;
    gy_off = g1_off;
    gz_off = g2_off;


    // start motorshield
    AFMS.begin(PWM_FREQ);
    
      
}


void loop() {
    update_angle();     

    // safety, if we are beyond saving just drop dead
    if(abs((targ_ang - ang_f)) > 20) 
    {
      go = 0;
      stop_motors();
    }
    else
    {
      compute_command(); // run PID

      // only update every so often because of inertia
      if(millis() - last_speed_ms > 50) 
      {
        last_speed_ms = millis();
        set_speed();
      }
    }
   

    

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);


    // serial debug, update status
    if(millis() - last_ms > 50)
    {
  /*     Serial.print(gy);
       Serial.print("      ");
       Serial.print(dt/1000000.0); 
       Serial.print("      ");
       Serial.print(ang);
       Serial.print("      ");*/
   /*    Serial.print(P*err);
       Serial.print(" , ");
       Serial.print(D*d_err);
       Serial.print(" , ");
       Serial.println(I*cum_err);
      
  /*     Serial.print("     ");
       Serial.println(acc_ang);*/
       Serial.print(ang_f);
       Serial.print(" , ");
        if(cmd < 255 && cmd > -255)
          Serial.println(cmd);
        else if (cmd < -255)
          Serial.println(-255);
        else
          Serial.println(255);
      
        last_ms = millis();
    }

    // check for serial start/stop command
    if(Serial.available())
    {
      char c = Serial.read();
      if(c == 's')
      {
        stop_motors();
        go = !go;
      }
    }

    // give some rest
    delay(3);
}






// ************************************** begin angle estimation


void update_angle()
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    us = micros();
    dt = us - last_us;
    last_us = us;

    correct_MPU_values(); // remove offsets

    gy_f = gy*deg_per_lsb;
    dt_f = dt/1000000.0;
    ang_f = integrate_f(ang_f, dt_f, gy_f);

    acc_ang = atan2(ax,az)*180/3.14159;
}

void correct_MPU_values()
{
  ax -= ax_off;
  ay -= ay_off;
  az -= az_off;
  gx -= gx_off;
  gy -= gy_off;
  gz -= gz_off;
}


float integrate_f(float ang, float dt, float gy)
{
  ang += gy*(dt);
  return ang;
}

// ************************************** end angle estimation


// ************************************** begin control
void compute_command()
{
  err = targ_ang - ang_f;
  d_err = err - prev_err;
  prev_err = err;
  cum_err += err;
  cmd = P*err + I*cum_err + D*d_err;
}

// ************************************** end control

// ************************************** begin motor
void stop_motors()
{
    m1->setSpeed(0);
    m2->setSpeed(0);
    m1->run(RELEASE);
    m2->run(RELEASE);
}


void set_speed()
{
  if(cmd > 0)
  {
    m1->run(FORWARD);
    m2->run(BACKWARD);
  }
  else
  {
    m1->run(BACKWARD);
    m2->run(FORWARD);
  }
  m1->setSpeed(min(SPEED_OFFSET+abs(cmd), 255));
  m2->setSpeed(min(SPEED_OFFSET+abs(cmd), 255));
  
}
// ************************************** end motor







// ************************************** begin not used anymore
/*

void print_values(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);
}


void print_floats(float ax, float ay, float az, float gx, float gy, float gz)
{
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);
}


int16_t integrate(int16_t ang, unsigned long dt, int16_t gy)
{
  ang += gy*(dt/1000000.0);
  return ang;
}

void convert_to_g()
{
  x = ax/G_from_MPU;
  y = ay/G_from_MPU;
  z = az/G_from_MPU;

  xx = gx/deg_from_MPU;
  yy = gy/deg_from_MPU;
  zz = gz/deg_from_MPU;
}

// ************************************** end not used anymore

*/
