#include <Encoder.h>
#include <BasicLinearAlgebra.h>
#include <Metro.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>

ros::NodeHandle nh;
std_msgs::Int16 enc_tick_count;
std_msgs::String kf_vals;

ros::Publisher encPub("encoder_ticks", &enc_tick_count);
ros::Publisher kfPub("kf_vals", &kf_vals);

using namespace BLA;

Metro sendPos = Metro(5);
Metro control = Metro(20);

const int PWM = 7;
//const int PWM = 9;
const int CW = 5;
const int CCW = 6;
const int SPD = 0;

const int SHTDWN1 = 10;
const int SHTDWN2 = 11;

float spd = 0;
float pos = 0;
float phi = 0;

float integrator = 0;
int iWup = 0;

int startup = 1;
int stopMC = 0;

int pix;
int flag = 0;

void updatePix(const std_msgs::Int16& yolo_cmd);

ros::Subscriber<std_msgs::Int16> yoloSub("yolo_cmd", &updatePix);

float motor_comm = 0;

int updateOnlyCount = 10;
int useOR = 0;

#define PI 3.1415926535897932384626433832795
#define ticks_to_rad 2*PI/102000
#define adc_to_volt 5.0/1023 
#define dt 0.02

Encoder myEnc(2,3);

float sigma = 0.4110;
//0.4110; //max jerk

//float focalLength = 948.8390198482;// webcam
float focalLength = 1969.717464;// arducam (zoom)

BLA::Matrix<2,2> R = {0.001*0.001, 0, 
                      0, 5*5};
BLA::Matrix<2,2> R1 = {0.001*0.001, 0,
                       0, PI/4*PI/4};
//BLA::Matrix<6,6> Q = {sq(sigma), 0, 0, 0, 0, 0, 
//                      0, sq(sigma), 0, 0, 0, 0, 
//                      0, 0, sq(dt)*sq(sigma), 0, 0, 0, 
//                      0, 0, 0, sq(dt)*sq(sigma), 0, 0, 
//                      0, 0, 0, 0, 0.25*sq(dt)*sq(dt)*sq(sigma), 0, 
//                      0, 0, 0, 0, 0, 0.25*sq(dt)*sq(dt)*sq(sigma)};
BLA::Matrix<6,6> Q = {0.25*dt*dt*dt*dt*sigma*sigma, 0, 0, 0, 0, 0, 
                      0, 0.25*dt*dt*dt*dt*sigma*sigma, 0, 0, 0, 0, 
                      0, 0, dt*dt*sigma*sigma, 0, 0, 0, 
                      0, 0, 0, dt*dt*sigma*sigma, 0, 0, 
                      0, 0, 0, 0, 10*dt*dt*sigma*sigma, 0, 
                      0, 0, 0, 0, 0, 10*dt*dt*sigma*sigma};
//BLA::Matrix<6,6> Q = {dt*sigma*sigma, 0, 0, 0, 0, 0, 
//                      0, dt*sigma*sigma, 0, 0, 0, 0, 
//                      0, 0, 1/3*dt*dt*dt*sigma*sigma, 0, 0, 0, 
//                      0, 0, 0, 1/3*dt*dt*dt*sigma*sigma, 0, 0, 
//                      0, 0, 0, 0, 1/20*dt*dt*dt*dt*dt*sigma*sigma, 0, 
//                      0, 0, 0, 0, 0, 1/20*dt*dt*dt*dt*dt*sigma*sigma};

//*Starting from Left of frame             
//BLA::Matrix<6,1> x = {0, 0, 0, 0, 0, -PI/8};
//*Starting from Rightof frame
//BLA::Matrix<6,1> x = {0, 0, 0, 0, 0, PI/8};
//Starting from centre of frame
BLA::Matrix<6,1> x = {0, 0, 0, 0.5, 0, 0};

//BLA::Matrix<6,6> F = {1, 0, 0, 0, 0, 0, 
//                      0, 1, 0, 0, 0, 0, 
//                      dt, 0, 1, 0, 0, 0, 
//                      0, dt, 0, 1, 0, 0, 
//                      0, 0, dt, 0, 1, 0, 
//                      0, 0, 0, dt, 0, 1};
BLA::Matrix<6,6> F = {1, 0, 0, 0, 0, 0, 
                      0, 1, 0, 0, 0, 0, 
                      dt, 0, 1, 0, 0, 0, 
                      0, dt, 0, 1, 0, 0, 
//                      0.5*dt*dt, 0, dt, 0, 1, 0, 
//                      0, 0.5*dt*dt, 0, dt, 0, 1};
                      0, 0, dt, 0, 1, 0, 
                      0, 0, 0, dt, 0, 1};                      
                      
BLA::Matrix<6,6> P = {(0.001*0.001)/(dt*dt), 0, 0, 0, 0, 0, 
                      0, (PI/3*PI/3)/(dt*dt), 0, 0, 0, 0, 
                      0, 0, 0.001*0.001/dt, 0, 0, 0, 
                      0, 0, 0, (PI/3*PI/3)/dt, 0, 0, 
                      0, 0, 0, 0, 0.001*0.001, 0, 
                      0, 0, 0, 0, 0, PI/3*PI/3};                 
BLA::Matrix<2,1> z;
BLA::Matrix<2,1> y;
BLA::Matrix<6,2> K;
BLA::Matrix<2,2> S;
BLA::Matrix<2,6> H;

BLA::Matrix<1,6> H2;
BLA::Matrix<1,1> S2;
BLA::Matrix<6,1> K2;

void setup() {
  // put your setup code here, to run once:
  pinMode(CW, OUTPUT);
  pinMode(CCW, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(SHTDWN1, INPUT);
  pinMode(SHTDWN2, INPUT);
  
  analogWrite(PWM,26);
  digitalWrite(CW, 0);
  digitalWrite(CCW, 0);    

  attachInterrupt(digitalPinToInterrupt(SHTDWN1), shutdownMC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SHTDWN2), shutdownMC, CHANGE);  

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(encPub);
  nh.advertise(kfPub);
  nh.subscribe(yoloSub);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (sendPos.check() == 1){
    enc_tick_count.data = -1*myEnc.read();
    encPub.publish(&enc_tick_count);
    nh.spinOnce();    
  }
  
  if ((stopMC ==  0) and (control.check() == 1)){
    nh.spinOnce();
    if (flag==1){ //Check if new yolo value has come in
      if (startup == 0){
        pix = constrain(pix,-640,640);
      }
      else{
       pix = constrain(pix,-320,320); 
      }
  
      //Predict Step:
      x = F*x;
      P = F*P*~F + Q;
      
      //Update with encoder and yolo measurements
      H = {0, 0, 0, 0, 1, 0,
           0, 0, 0, 0, focalLength/sq(cos((x(4)-x(5)))), -focalLength/sq(cos((x(4)-x(5))))};
      S = H*P*~H + R;
      BLA::Matrix<2,2> S_inv = {S(1,1), -S(0,1), -S(1,0), S(0,0)};
      float S_inv_sf = 1/(S(0,0)*S(1,1) - S(0,1)*S(1,0));
      K = (P*~H*S_inv) * S_inv_sf ;
      
      z(0) = myEnc.read()*ticks_to_rad; //rad
      z(1) = pix;
      
      y(0) = z(0) - x(4);
      y(1) = z(1) - (-focalLength)*tan(x(5)-x(4));

      //Check for outliers - If residual > 3*sigma, then measurement is an outlier, so drop residual to 0
//      if ((useOR == 1) and (y(1) > 3*5*5)){
//        y(1) = 0;
//      }

      x += K*y;      
      P = P-K*H*P;
      x(5) = constrain(x(5), -PI/3, PI/3);
      x(4) = constrain(x(4), -PI/3, PI/3);
//      x(5) = constrain(x(5), 0, 2*PI/3);
//      x(4) = constrain(x(4), 0, 2*PI/3);

      std::stringstream xSStr;
      xSStr << "[" << x(0) << ", " << x(1) << ", " << x(2) << ", " << x(3) << ", " << x(4) << ", " << x(5) << "]";
      std::string xStr = xSStr.str();
      const char* p = xStr.c_str();
      kf_vals.data =  p;
      kfPub.publish(&kf_vals);
      nh.spinOnce();
      
      //Motor command (position) = phi (subject center in world coords)
      motor_comm = x(5);      
      
      updateOnlyCount = 0;
      useOR = 1;
      startup = 0;
      flag=0;
    }
    else{
      //Check if last yolo update was less than 10 samples ago
      if (updateOnlyCount < 10 and startup == 0){
        //Predict
        x = F*x;
        P = F*P*~F + Q; 
        
        //Update with encoder and pseudo measurements
        H = {0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1};
        S = H*P*~H + R1;
        BLA::Matrix<2,2> S_inv3 = {S(1,1), -S(0,1), -S(1,0), S(0,0)};
        float S_inv_sf = 1/(S(0,0)*S(1,1) - S(0,1)*S(1,0));
        K = (P*~H*S_inv3) * S_inv_sf ;
        
        z(0) = myEnc.read()*ticks_to_rad; //rad
        y(0) = z(0) - x(4);
        z(1) = x(4);
        y(1) = z(1) - x(5);
      
        x += K*y;
        P = P-K*H*P;
        x(5) = constrain(x(5), -PI/3, PI/3);
        x(4) = constrain(x(4), -PI/3, PI/3);
//        x(5) = constrain(x(5), 0, 2*PI/3);
//        x(4) = constrain(x(4), 0, 2*PI/3); 

        std::stringstream xSStr;
        xSStr << "[" << x(0) << ", " << x(1) << ", " << x(2) << ", " << x(3) << ", " << x(4) << ", " << x(5) << "]";
        std::string xStr = xSStr.str();
        const char* p = xStr.c_str();
        kf_vals.data =  p;
        kfPub.publish(&kf_vals);
        nh.spinOnce();
        
        //Motor command (position) = phi (subject center in world coords)
        motor_comm = x(5);
        
        updateOnlyCount = updateOnlyCount + 1;
      }
      else{useOR = 0;} //If target is lost for too long, do nothing
    }
            
    //Saturate motor position to prevent full rotations
    //Centre
    if (motor_comm < -PI/3){
      motor_comm = -PI/3;
    }
    else if (motor_comm > PI/3){
      motor_comm = PI/3;
    }
//    //Left
//    if (motor_comm < 0){
//      motor_comm = 0;
//      x(0) = 0;
//      x(1) = 0;
//      x(2) = 0;
//      x(3) = 0;
//    }
//    else if (motor_comm > 1.7*PI/3){
//      motor_comm = 1.7*PI/3;
//      x(0) = 0;
//      x(1) = 0;
//      x(2) = 0;
//      x(3) = 0;
//    }
    //Right
//        if (motor_comm < -2*PI/3){
//     motor_comm = -2*PI/3;
//    }
//    else if (motor_comm > 0){
//      motor_comm = 0;
//    }

    spd = ((analogRead(SPD)*adc_to_volt+0.1)*3040 - 6080)/51 * (-PI/30); //rad/s 0.1volt offset (pin reads 0.1 less than it should)
    pos = myEnc.read()*ticks_to_rad; //rad

    //Calculate motor current using FSF controller
    float current = iCMD(motor_comm, pos, spd, 0.02);

   //Scale from current between 0 to 1 amp to duty cycle between 10 and 90%
    //float duty_cycle_100 = (abs(current)/1*(90-10) + 10) ;
    //Scale from current between 0 to 1.5 amp to duty cycle between 10 and 90%
    float duty_cycle_100 = (abs(current)/1.5*(90-10) + 10) ;
    //Scale dc from 10 to 90 to dc from 25 to 230
    float duty_cycle_255 = (duty_cycle_100-10)/(90-10)*(230-25)+25;
    int duty_cycle_int = int(duty_cycle_255);

    //Set motor direction pins and current
    if (current > 0){
      digitalWrite(CCW, 0);
      digitalWrite(CW, 1);
      analogWrite(PWM, duty_cycle_int);
    }
    else if (current < 0){
      digitalWrite(CW, 0);
      digitalWrite(CCW, 1);
      analogWrite(PWM, duty_cycle_int);    
    }
    else{
      digitalWrite(CW, 0);
      digitalWrite(CCW, 0);
      analogWrite(PWM, 25);
    }
  }
  //Stop everything if endstops are triggered
  else if(stopMC == 1){}
}

float iCMD(float cmd, float pos, float vel, float sT){
  static float iMax = 1.5;
  float current;

  //Current best (old -> GoPros)
//  int kInt = 14.6307;
//  int kPos = 6.0918;
//  int kVel = 0.9208;  

  //New (with thia lenses) -> good
  int kInt = 12.1221;
  int kPos = 4.448;
  int kVel = 0.5792;
  
  float error = cmd - pos;
  if (iWup != 1){
    integrator = integrator + error*sT;
  }
  current = kInt*integrator - kPos*pos - kVel*vel;
  if (current < -iMax){
    iWup = 1;
    current = -iMax;
  }
  else if (current > iMax){
    iWup = 1;
    current = iMax;
  }
  else{
    iWup = 0;
  }
  
  return current;
}

void shutdownMC(){
  stopMC = 1;
  analogWrite(PWM, 25);
}

void updatePix(const std_msgs::Int16& yolo_cmd){
  pix = yolo_cmd.data;
  flag=1;
}
