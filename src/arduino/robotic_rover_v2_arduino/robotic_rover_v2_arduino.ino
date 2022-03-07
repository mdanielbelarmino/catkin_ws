/*Data Format
   LeftMotor Speed, RightMotor Speed, Headlight Intensity, Picker(bool), AUX1, AUX2
    255,255,50,1,0,0%
*/

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>

#define pwm_pinR2 9
#define dir_pinR2 17
#define pwm_pinR 11
#define dir_pinR 23
#define pwm_pinL2 8
#define dir_pinL2 16
#define pwm_pinL 10
#define dir_pinL 22

#define front_tof_pin A0
#define rear_tof_pin A1

#define MAX_SPEED 22
#define LOOPTIME 100
#define MAX_PWM 85
#define ROT_SPEED 20

#define prop 0.05

int t2 = 0;
int t1;
int period;

volatile int pulseL = 0;
volatile int pulseR = 0;
volatile int pulseL2 = 0;
volatile int pulseR2 = 0;
double rpmL;
double rpmR;
double rpmL2;
double rpmR2;
double curr_spdL; // cm/s
double des_spdL; // cm/s
double curr_spdR; // cm/s
double des_spdR; // cm/s
double curr_spdL2; // cm/s
double des_spdL2; // cm/s
double curr_spdR2; // cm/s
double des_spdR2; // cm/s
int16_t pwmL = 0;
int16_t pwmR = 0;
int16_t pwmL2 = 0;
int16_t pwmR2 = 0;

// Add direction parameter here

double gen_speed = MAX_SPEED;

void isrL() //interrupt service routine
{
  pulseL++;
}

void isrR() //interrupt service routine
{
  pulseR++;
}

void isrL2() //interrupt service routine
{
  pulseL2++;
}

void isrR2() //interrupt service routine
{
  pulseR2++;
}

ros::NodeHandle  nh;
String rec;
int state = 0;

void messageCb( const geometry_msgs::Twist& cmd)
{

  float spd = cmd.linear.x;
  float ang = cmd.angular.z;

  if (spd > 0 && ang == 0) //forward
  {
    des_spdL = gen_speed;
    des_spdR = gen_speed;
    des_spdL2 = gen_speed;
    des_spdR2 = gen_speed;
    digitalWrite(dir_pinL, HIGH); // forward
    digitalWrite(dir_pinR, LOW); // forward
    digitalWrite(dir_pinL2, HIGH); // forward
    digitalWrite(dir_pinR2, LOW); // forward
    state = 0;
  }
  else if (spd < 0 && ang == 0) //reverse
  {
    des_spdL = gen_speed;
    des_spdR = gen_speed;
    des_spdL2 = gen_speed;
    des_spdR2 = gen_speed;
    digitalWrite(dir_pinL, LOW);
    digitalWrite(dir_pinR, HIGH);
    digitalWrite(dir_pinL2, LOW);
    digitalWrite(dir_pinR2, HIGH);
    state = 1;
  }

  else if (ang < 0 && spd == 0) // right
  {
    des_spdL = ROT_SPEED;
    des_spdR = ROT_SPEED;
    des_spdL2 = ROT_SPEED;
    des_spdR2 = ROT_SPEED;
    digitalWrite(dir_pinL, HIGH);
    digitalWrite(dir_pinR, HIGH);
    digitalWrite(dir_pinL2, HIGH);
    digitalWrite(dir_pinR2, HIGH);
    state = 2;
  }
  else if ( ang > 0 && spd == 0) // left
  {
    des_spdL = ROT_SPEED;
    des_spdR = ROT_SPEED;
    des_spdL2 = ROT_SPEED;
    des_spdR2 = ROT_SPEED;
    digitalWrite(dir_pinL, LOW);
    digitalWrite(dir_pinR, LOW);
    digitalWrite(dir_pinL2, LOW);
    digitalWrite(dir_pinR2, LOW);
    state = 3;
  }
  
  // code if both spd and ang has value
  
  else if ( ang < 0 && spd > 0) // right forward
  {
    des_spdL = ROT_SPEED;
    des_spdR = ROT_SPEED/4;
    des_spdL2 = ROT_SPEED;
    des_spdR2 = ROT_SPEED/4;
    digitalWrite(dir_pinL, HIGH);
    digitalWrite(dir_pinR, HIGH);
    digitalWrite(dir_pinL2, HIGH);
    digitalWrite(dir_pinR2, HIGH);
    state = 4;
  }

    else if ( ang > 0 && spd > 0) // left forward
  {
    des_spdL = ROT_SPEED/4;
    des_spdR = ROT_SPEED;
    des_spdL2 = ROT_SPEED/4;
    des_spdR2 = ROT_SPEED;
    digitalWrite(dir_pinL, LOW);
    digitalWrite(dir_pinR, LOW);
    digitalWrite(dir_pinL2, LOW);
    digitalWrite(dir_pinR2, LOW);
    state = 5;
  }

  else if ( ang > 0 && spd < 0) // right reverse
  {
    des_spdL = ROT_SPEED;
    des_spdR = ROT_SPEED/4;
    des_spdL2 = ROT_SPEED;
    des_spdR2 = ROT_SPEED/4;
    digitalWrite(dir_pinL, LOW);
    digitalWrite(dir_pinR, LOW);
    digitalWrite(dir_pinL2, LOW);
    digitalWrite(dir_pinR2, LOW);
    state = 4;
  }

    else if ( ang < 0 && spd < 0) // left reverse
  {
    des_spdL = ROT_SPEED/4;
    des_spdR = ROT_SPEED;
    des_spdL2 = ROT_SPEED/4;
    des_spdR2 = ROT_SPEED;
    digitalWrite(dir_pinL, HIGH);
    digitalWrite(dir_pinR, HIGH);
    digitalWrite(dir_pinL2, HIGH);
    digitalWrite(dir_pinR2, HIGH);
    state = 5;
  }
  
  else if (spd == 0)
  {
    des_spdL = 0;
    des_spdR = 0;
    des_spdL2 = 0;
    des_spdR2 = 0;
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb );
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type


void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(speed_pub);


  attachInterrupt(4, isrR2, RISING);
  attachInterrupt(2, isrR, RISING);
  attachInterrupt(5, isrL2, RISING);
  attachInterrupt(3, isrL, RISING);

  pinMode(pwm_pinL, OUTPUT);
  pinMode(dir_pinL, OUTPUT);
  pinMode(pwm_pinR, OUTPUT);
  pinMode(dir_pinR, OUTPUT);
  pinMode(pwm_pinL2, OUTPUT);
  pinMode(dir_pinL2, OUTPUT);
  pinMode(pwm_pinR2, OUTPUT);
  pinMode(dir_pinR2, OUTPUT);
  pinMode(front_tof_pin, INPUT);
  pinMode(rear_tof_pin, INPUT);

  analogWrite(pwm_pinR, 0);
  analogWrite(pwm_pinL, 0);
  analogWrite(pwm_pinR2, 0);
  analogWrite(pwm_pinL2, 0);

  digitalWrite(dir_pinL, HIGH); // forward
  digitalWrite(dir_pinR, LOW); // forward
  digitalWrite(dir_pinL2, HIGH); // forward
  digitalWrite(dir_pinR2, LOW); // forward

}

int ctr = 0;

void loop()
{

  detachInterrupt(4);           //detaches the interrupt pin 19 L old
  detachInterrupt(2);           //detaches the interrupt pin 21 R
  detachInterrupt(5);           //detaches the interrupt pin 18 L2
  detachInterrupt(3);           //detaches the interrupt pin 20 R2

  t1 = millis();    //finds the time
  bool front_tof = digitalRead(front_tof_pin);
  bool rear_tof = digitalRead(rear_tof_pin);

  period = t1 - t2;
  if (period >= 10)
  {

    float lp = pulseL;
    float rp = pulseR;
    float lp2 = pulseL2;
    float rp2 = pulseR2;

    rpmL = (lp / 0.43) * 60;
    rpmR = (rp / 0.43) * 60;
    rpmL2 = (lp2 / 0.43) * 60;
    rpmR2 = (rp2 / 0.43) * 60;

    curr_spdL = (rpmL / 60) * (2 * PI * 0.08) * 100;
    curr_spdR = (rpmR / 60) * (2 * PI * 0.08) * 100;
    curr_spdL2 = (rpmL2 / 60) * (2 * PI * 0.08) * 100;
    curr_spdR2 = (rpmR2 / 60) * (2 * PI * 0.08) * 100;

    pulseL = 0;
    pulseR = 0;
    pulseL2 = 0;
    pulseR2 = 0;

    t2 = t1;           //saves the current time
    int errL = des_spdL - curr_spdL;
    pwmL = pwmL + (errL * prop); // proportional

    int errR = des_spdR - curr_spdR;
    pwmR = pwmR + (errR * prop); // proportional

    int errL2 = des_spdL2 - curr_spdL2;
    pwmL2 = pwmL2 + (errL2 * prop); // proportional

    int errR2 = des_spdR2 - curr_spdR2;
    pwmR2 = pwmR2 + (errR2 * prop); // proportional

    pwmL = (pwmL > MAX_PWM) ? MAX_PWM : pwmL;
    pwmR = (pwmR > MAX_PWM) ? MAX_PWM : pwmR;
    pwmL2 = (pwmL2 > MAX_PWM) ? MAX_PWM : pwmL2;
    pwmR2 = (pwmR2 > MAX_PWM) ? MAX_PWM : pwmR2;

    pwmL = (pwmL < 0) ? 0 : pwmL;
    pwmR = (pwmR < 0) ? 0 : pwmR;
    pwmL2 = (pwmL2 < 0) ? 0 : pwmL2;
    pwmR2 = (pwmR2 < 0) ? 0 : pwmR2;

    analogWrite(pwm_pinL, pwmL);      //sets the desired speed
    analogWrite(pwm_pinR, pwmR);      //sets the desired speed
    analogWrite(pwm_pinL2, pwmL2);      //sets the desired speed
    analogWrite(pwm_pinR2, pwmR2);      //sets the desired speed

    if(des_spdL == 0) 
      pwmL = 0;
    if(des_spdL2 == 0) 
      pwmL2 = 0;
    if(des_spdR == 0) 
      pwmR = 0;
    if(des_spdR2 == 0) 
      pwmR2 = 0;

    publishSpeed(LOOPTIME);
  }

  attachInterrupt(4, isrR2, RISING);
  attachInterrupt(2, isrR, RISING);
  attachInterrupt(5, isrL2, RISING);
  attachInterrupt(3, isrL, RISING);

  nh.spinOnce();


}

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = state;    //left wheel speed (in m/s)
  speed_msg.vector.y = pwmL2;   //right wheel speed (in m/s)
  speed_msg.vector.z = des_spdL; //time / 1000;       //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  //  nh.loginfo("Publishing odometry");
}
