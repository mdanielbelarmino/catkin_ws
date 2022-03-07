/*Data Format
   LeftMotor Speed, RightMotor Speed, Headlight Intensity, Picker(bool), AUX1, AUX2
    255,255,50,1,0,0%
*/

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>

#define pwm_pinL 5
#define dir_pinL 4
#define pwm_pinR 6
#define dir_pinR 7

#define MAX_SPEED 10
#define LOOPTIME 100

int t2 = 0;
int t1;
int period;

volatile unsigned int pulseL = 0;
volatile unsigned int pulseR = 0;
double rpmL;
double rpmR;
double curr_spdL; // cm/s
double des_spdL; // cm/s
double curr_spdR; // cm/s
double des_spdR; // cm/s
uint16_t pwmL = 0;
uint16_t pwmR = 0;

// Kinematics Constant
const float wheel_base = 0.26;
const float wheel_radius = 0.055;

double gen_speed = MAX_SPEED;

void isrL() //interrupt service routine
{
  pulseL++;
}

void isrR() //interrupt service routine
{
  pulseR++;
}

ros::NodeHandle  nh;
String rec;
// for debugging purposes
int state = 0;


void messageCb( const geometry_msgs::Twist& cmd)
{

  double linear_velocity = cmd.linear.x;
  double angular_velocity = cmd.angular.z;

  angular_velocity = (angular_velocity < 0.0001 && angular_velocity > -0.0001) ? 0.0 : angular_velocity;
  linear_velocity = (linear_velocity < 0.0001 && linear_velocity > -0.0001) ? 0.0 : linear_velocity;

  double Vl = (linear_velocity - angular_velocity * wheel_base / 2) / wheel_radius;
  double Vr = (linear_velocity + angular_velocity * wheel_base / 2) / wheel_radius;

  des_spdL = abs(Vl);
  des_spdR = abs(Vr);

  if (linear_velocity > 0.0 && angular_velocity == 0.0) //forward
  {
    digitalWrite(dir_pinL, HIGH); // forward
    digitalWrite(dir_pinR, LOW); // forward
    state = 0;
  } 
  else if (linear_velocity < 0.0 && angular_velocity == 0.0) //reverse
  {
    digitalWrite(dir_pinL, LOW);
    digitalWrite(dir_pinR, HIGH);
    state = 1;
  }

  else if (angular_velocity < 0.0 && linear_velocity == 0.0) // turn right
  {
    digitalWrite(dir_pinL, HIGH);
    digitalWrite(dir_pinR, HIGH);
    state = 2;
  }
  else if ( angular_velocity > 0.0 && linear_velocity == 0.0) // turn left
  {
    digitalWrite(dir_pinL, LOW);
    digitalWrite(dir_pinR, LOW);
    state = 3;
  }

  // code if both linear_velocity and angular_velocity has value

  else if ( angular_velocity < 0.0 && linear_velocity > 0.0) // steer right
  {
    digitalWrite(dir_pinL, HIGH); // forward
    digitalWrite(dir_pinR, LOW); // forward
    state = 4;
  }

  else if ( angular_velocity > 0.0 && linear_velocity > 0.0) // steer left
  {
    digitalWrite(dir_pinL, HIGH); // forward
    digitalWrite(dir_pinR, LOW); // forward
    state = 5;
  }

  else if ( angular_velocity > 0.0 && linear_velocity < 0.0) // steer right reverse
  {
    digitalWrite(dir_pinL, LOW);
    digitalWrite(dir_pinR, HIGH);
    state = 6;
  }

  else if ( angular_velocity < 0.0 && linear_velocity < 0.0) // steer left reverse
  {
    digitalWrite(dir_pinL, LOW);
    digitalWrite(dir_pinR, HIGH);
    state = 7;
  }

  else if (linear_velocity == 0.0 && angular_velocity == 0.0) // stop
  {
    des_spdL = 0;
    des_spdR = 0;
    state = 8;
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

  attachInterrupt(0, isrL, RISING); //attaching the interrupt to pin3
  attachInterrupt(1, isrR, RISING); //attaching the interrupt to pin3

  pinMode(pwm_pinL, OUTPUT);
  pinMode(dir_pinL, OUTPUT);
  pinMode(pwm_pinR, OUTPUT);
  pinMode(dir_pinR, OUTPUT);

  analogWrite(pwm_pinR, 0);
  analogWrite(pwm_pinL, 0);
  digitalWrite(dir_pinL, HIGH);
  digitalWrite(dir_pinR, LOW);
}

void loop()
{
  detachInterrupt(0);           //detaches the interrupt
  detachInterrupt(1);           //detaches the interrupt

  t1 = millis();    //finds the time

  period = t1 - t2;
  if (period >= 10)
  {

    double lp = pulseL;
    double rp = pulseR;
    rpmL = (lp / 43) * 60;
    rpmR = (rp / 43) * 60;

    curr_spdL = (rpmL / 60) * (2 * PI * 0.055) * 100;
    curr_spdR = (rpmR / 60) * (2 * PI * 0.055) * 100;

    pulseL = 0;
    pulseR = 0;

    t2 = t1;           //saves the current time
    double errL = des_spdL - curr_spdL;
    pwmL = pwmL + (errL * 2); // proportional

    double errR = des_spdR - curr_spdR;
    pwmR = pwmR + (errR * 2); // proportional

    pwmL = (pwmL > 255) ? 255 : pwmL;
    pwmR = (pwmR > 255) ? 255 : pwmR;
    pwmL = (pwmL < 0) ? 0 : pwmL;
    pwmR = (pwmR < 0) ? 0 : pwmR;

    analogWrite(pwm_pinL, pwmL);      //sets the desired speed
    analogWrite(pwm_pinR, pwmR);      //sets the desired speed
    publishSpeed(LOOPTIME);
  }

  attachInterrupt(0, isrL, RISING);
  attachInterrupt(1, isrR, RISING);

  nh.spinOnce();


}

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = des_spdL;    //left wheel speed (in m/s)
  speed_msg.vector.y = des_spdR;   //right wheel speed (in m/s)
  speed_msg.vector.z = time / 1000;       //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  //  nh.loginfo("Publishing odometry");
}
