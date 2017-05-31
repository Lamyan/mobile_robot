
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <PWM.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include "robot_specs.h"
#include <Wire.h>
#define encodPinA1      2     // encoder A pin
#define encodPinA2      3
#define LOOPTIME        100   // PID loop time(ms)
#define SMOOTH      10

#define sign(x) (x > 0) - (x < 0)
unsigned long lastMilli = 0;       // loop timing
unsigned long lastMilliPub = 0;
double rpm_req1 = 0;
double rpm_req2 = 0;
double rpm_act1 = 0;
double rpm_act2 = 0;
double rpm_req1_smoothed = 0;
double rpm_req2_smoothed = 0;
int PWM_val1 = 0;
int PWM_val2 = 0;
volatile long count1 = 0;          // rev counter
volatile long count2 = 0;
long countAnt1 = 0;
long countAnt2 = 0;
float Kp =   0.5;
float Kd =   0;
float Ki =   0;
ros::NodeHandle nh;

void handle_cmd( const geometry_msgs::Twist& cmd_msg) {
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;
  if (z == 0) {     // go straight
    // convert m/s to rpm
    rpm_req1 = x * 60 / (pi * wheel_diameter);
    rpm_req2 = rpm_req1;
  }
  else if (x == 0) {
    // convert rad/s to rpm
    rpm_req2 = z * track_width * 60 / (wheel_diameter * pi * 2);
    rpm_req1 = -rpm_req2;
  }
  else {
    rpm_req1 = x * 60 / (pi * wheel_diameter) - z * track_width * 60 / (wheel_diameter * pi * 2);
    rpm_req2 = x * 60 / (pi * wheel_diameter) + z * track_width * 60 / (wheel_diameter * pi * 2);
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle_cmd);
geometry_msgs::Vector3Stamped rpm_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg);
ros::Time current_time;
ros::Time last_time;

void setup() {
  Serial.begin(9600);  // create with the default frequency 1.6KHz
  count1 = 0;
  count2 = 0;
  countAnt1 = 0;
  countAnt2 = 0;
  rpm_req1 = 0;
  rpm_req2 = 0;
  rpm_act1 = 0;
  rpm_act2 = 0;
  PWM_val1 = 0;
  PWM_val2 = 0;
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(sub);
  nh.advertise(rpm_pub);

  pinMode(2, INPUT);
  digitalWrite(2, HIGH);                // turn on pullup resistor
  attachInterrupt(0, encoder1, RISING);

  pinMode(3, INPUT);
  digitalWrite(3, HIGH);                // turn on pullup resistor
  attachInterrupt(1, encoder2, RISING);

  analogWrite(8, 0); //motor1->setSpeed(0);
  analogWrite(13, 0); // motor2->setSpeed(0);
  digitalWrite(9, HIGH);//motor1->run(FORWARD);
  digitalWrite(10, LOW);
  digitalWrite(9, LOW);//motor1->run(RELEASE);
  digitalWrite(10, LOW);
  digitalWrite(11, HIGH);
  digitalWrite(12, LOW);//motor2->run(FORWARD);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  //motor2->run(RELEASE);
}

void loop() {
  nh.spinOnce();
  unsigned long time = millis();
  if (time - lastMilli >= LOOPTIME)   {  // enter tmed loop
    getMotorData(time - lastMilli);
    PWM_val1 = updatePid(1, PWM_val1, rpm_req1, rpm_act1);
    PWM_val2 = updatePid(2, PWM_val2, rpm_req2, rpm_act2);

    if (PWM_val1 > 0) {
      digitalWrite(9, HIGH);
      digitalWrite(10, LOW);//direction1 = FORWARD;
    }
    else if (PWM_val1 < 0) {
      digitalWrite(9, LOW);
      digitalWrite(10, HIGH);//Drection1 = BACKWARD;
    } 
    if (rpm_req1 == 0) {
      digitalWrite(9, LOW);
      digitalWrite(10, LOW); //direction1 = RELEASE;
    }
    
    if (PWM_val2 > 0){
      digitalWrite(11, HIGH);
      digitalWrite(12, LOW);//direction2 = FORWARD;
    }
    else if (PWM_val2 < 0) {
      digitalWrite(11, LOW);
      digitalWrite(12, HIGH);//Drection2 = BACKWARD;
    } 
    if (rpm_req2 == 0) {
      digitalWrite(11, LOW);
      digitalWrite(12, LOW); //direction2 = RELEASE;
    }
    

    analogWrite(8, abs(PWM_val1) );//motor1->setSpeed(abs(PWM_val1));
    analogWrite(13, abs(PWM_val2) );//motor2->setSpeed(abs(PWM_val2));

    publishRPM(time - lastMilli);
    lastMilli = time;
  }
  if (time - lastMilliPub >= LOOPTIME) {
    //  publishRPM(time-lastMilliPub);
    lastMilliPub = time;
  }
}

void getMotorData(unsigned long time)  {
  rpm_act1 = double((count1 - countAnt1) * 60 * 1000) / double(time * 13 * 120);
  rpm_act2 = double((count2 - countAnt2) * 60 * 1000) / double(time * 13 * 120);
  countAnt1 = count1;
  countAnt2 = count2;
}

int updatePid(int id, int command, double targetValue, double currentValue) {
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;

  error = targetValue - currentValue;
  if (id == 1) {
    int_error1 += error;
    pidTerm = Kp * error + Kd * (error - last_error1) + Ki * int_error1;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp * error + Kd * (error - last_error2) + Ki * int_error2;
    last_error2 = error;
  }
  new_pwm = constrain(double(command) * MAX_RPM / 255 + pidTerm, -MAX_RPM, MAX_RPM);
  new_cmd = 255 * new_pwm / MAX_RPM;
  return int(new_cmd);
}

void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time) / 1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}

void encoder1() {
 count1+=5;//  if (digitalRead(encodPinA1) == digitalRead(encodPinB1)) count1++;
 // else count1--;
}
void encoder2() {
 count2+=4;// if (digitalRead(encodPinA2) == digitalRead(encodPinB2)) count2--;
 // else count2++;
}
