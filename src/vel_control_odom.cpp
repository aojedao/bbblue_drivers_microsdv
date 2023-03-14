#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <cmath>


#include <rc/motor.h>
#include <rc/encoder.h>
#include <rc/encoder_pru.h>
#include <rc/encoder_eqep.h>
#include <rc/mpu.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

//MPU values

// bus for Robotics Cape and BeagleboneBlue is 2
// change this for your platform
#define I2C_BUS 2

// possible modes, user selected with command line arguments
typedef enum g_mode_t{
        G_MODE_RAD,
        G_MODE_DEG,
        G_MODE_RAW
} g_mode_t;
typedef enum a_mode_t{
        A_MODE_MS2,
        A_MODE_G,
        A_MODE_RAW
} a_mode_t;

static int enable_magnetometer = 0;
static int enable_thermometer = 0;
static int enable_warnings = 0;
static int running = 0;

// global variables
ros::Time g_msg_received;
bool g_driving = 0;
int g_left_motor;      // param default 1
int g_right_motor;     // param default 2
double g_maxspeed;     // param default 0.4
double g_minspeed;     // param default 0.1
double g_turnspeed;    // param default 1
double g_wheelbase;    // param default 0.2
double g_duty_factor;  // param default 2.0
int g_rate;            // param default 10

double vx = 0;
double vy = 0;
double vth = 0;

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  g_msg_received = ros::Time::now();
  ROS_INFO("cmd_vel Linear: [%f, %f, %f] Angular: [%f, %f, %f]", cmd_vel->linear.x, cmd_vel->linear.y,
           cmd_vel->linear.z, cmd_vel->angular.x, cmd_vel->angular.y, cmd_vel->angular.z);

  vx = cmd_vel->linear.x;
  vy = cmd_vel->linear.y;
  vth = cmd_vel->angular.z;

  if (vx > g_maxspeed)
  {
    ROS_INFO("Velocity %f larger than %f! Limiting speed to %f.", vx, g_maxspeed, g_maxspeed);
    vx = g_maxspeed;
  }


  /*
  https://people.cs.umu.se/thomash/reports/KinematicsEquationsForDifferentialDriveAndArticulatedSteeringUMINF-11.19.pdf
  http://robotsforroboticists.com/drive-kinematics/
  
  velocity_left_cmd = (linear_velocity – angular_velocity * WHEEL_BASE / 2.0)/WHEEL_RADIUS;
  velocity_right_cmd = (linear_velocity + angular_velocity * WHEEL_BASE / 2.0)/WHEEL_RADIUS;

  self.right = 1.0 * self.dx + self.dr * self.w / 2
  self.left = 1.0 * self.dx - self.dr * self.w / 2

  */
 
}

int main(int argc, char** argv)
{
  // ROS and node initialize
  ros::init(argc, argv, "vel_control_odom");

  ros::NodeHandle n;

  ROS_INFO("Initializing node %s in namespace: %s", ros::this_node::getName().c_str(),
           ros::this_node::getNamespace().c_str());

  g_msg_received = ros::Time::now();

  // get parameter

  int cmd_vel_timeout;
  std::string base_frame_id_;

  // n.param("~timeout", cmd_vel_timeout, 5);
  ros::param::param("~timeout", cmd_vel_timeout, 5);
  ros::param::param("~left_motor", g_left_motor, 1);
  ros::param::param("~right_motor", g_right_motor, 2);
  ros::param::param("~maxspeed", g_maxspeed, 0.8);
  ros::param::param("~minspeed", g_minspeed, 0.1);
  ros::param::param("~wheelbase", g_wheelbase, 0.16);
  ros::param::param("~turnspeed", g_turnspeed, 1.0);
  ros::param::param("~duty_factor", g_duty_factor, 1.0);
  ros::param::param("~rate", g_rate, 10);

  if (g_left_motor < 1 or g_left_motor > 4 or g_right_motor < 1 or g_right_motor > 4)
  {
    ROS_ERROR("ERROR: Wrong parameter: left_motor/right_motor must be between 1-4");
    return -1;
  }

  n.param<std::string>("base_frame_id", base_frame_id_, "base_link");

  // initialize encoder hardware first
  if(rc_encoder_eqep_init()){
    ROS_ERROR("Initialize encoder FAILED");
    //fprintf(stderr,"ERROR: failed to run rc_encoder_init\n");
    return -1;
  }
  ROS_INFO("Initialize encoder: OK");

  // initialize motor hardware first
  int pwm_freq_hz = RC_MOTOR_DEFAULT_PWM_FREQ;  // 25000
  if (rc_motor_init_freq(pwm_freq_hz))
  {
    ROS_ERROR("Initialize motor %d and %d with %d: FAILED", g_left_motor, g_right_motor, pwm_freq_hz);
    return -1;
  }
  ROS_INFO("Initialize motor %d and %d with %d: OK", g_left_motor, g_right_motor, pwm_freq_hz);


  // odometry publisher
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  tf::TransformBroadcaster odom_broadcaster;
  // cmd_vel subscriber
  ros::Subscriber sub = n.subscribe("cmd_vel", 100, cmd_velCallback);
 
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  //odom variables

  //meters per tick
  double mpt=0.000169162;
  //La distancia entre la llanta y el centro del robot es de 0.08 m
  double robot_radius=0.08;
  double old_left_encoder_pos=0;
  double old_right_encoder_pos=0;
  double actual_left_encoder_pos;
  double actual_right_encoder_pos;
  double dr=0;
  double dl=0;
  double delta_dr=0;
  double delta_dl=0;

  double delta_theta=0;
  double delta_distance=0;
  double distance=0;
  double delta_theta_gyro=0;
  double old_gyrox=0;
  double old_gyroy=0;
  double old_gyroz=0;


  //MPU variables and configurations
  rc_mpu_data_t data; //struct to hold new data
  g_mode_t g_mode = G_MODE_DEG; // gyro default to degree mode.
  a_mode_t a_mode = A_MODE_MS2; // accel default to m/s^2

  rc_mpu_config_t conf = rc_mpu_default_config();
  conf.i2c_bus = I2C_BUS;
  conf.enable_magnetometer = enable_magnetometer;
  conf.show_warnings = enable_warnings;

  if(rc_mpu_initialize(&data, conf)){
        fprintf(stderr,"rc_mpu_initialize_failed\n");
        return -1;
}

  // %EndTag(SUBSCRIBER)%

  ROS_INFO("Node is up and Subsciber started");

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  // %Tag(SPIN)%

  ros::Rate r(g_rate);

  while (ros::ok())
  {
    ros::spinOnce();

    current_time = ros::Time::now();

    // stopping motor when no message received within timeout

    if (g_driving && (ros::Time::now().toSec() - g_msg_received.toSec()) > cmd_vel_timeout)

    {
      ROS_INFO("TIMEOUT: No cmd_vel received: setting motors to 0");

      // looks like 0 for all motors doesn't work
      // rc_motor_set(0,0);
      rc_motor_set(g_left_motor, 0);
      rc_motor_set(g_right_motor, 0);

      g_driving = 0;
    }

    actual_right_encoder_pos=rc_encoder_read(2)*(-1);
    actual_left_encoder_pos=rc_encoder_read(3);

    delta_dr=(actual_right_encoder_pos-old_right_encoder_pos)*mpt;
    delta_dl=(actual_left_encoder_pos-old_left_encoder_pos)*mpt;

    dr=dr+delta_dr;
    dl=dl+delta_dl;



    delta_theta=(delta_dr-delta_dl)/(2*robot_radius);
    delta_distance=(delta_dr+delta_dl)/2;
    distance=distance+delta_distance;

    old_left_encoder_pos=actual_left_encoder_pos;
    old_right_encoder_pos=actual_right_encoder_pos;
// compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

    double vel_x=delta_dr/dt;
    double vel_y=delta_dl/dt;
    double vel_th=(delta_dl-delta_dr)/dt;


    double delta_x = delta_distance*(double)cos(th)*dt*10;
    double delta_y = delta_distance*(double)sin(th)*dt*10;
    double delta_th = (delta_dl-delta_dr)/(2*robot_radius);

    std::cout << "El valor de vel_x es: "<<vel_x<<"m | El valor de vel_y es: "<<vel_y<<"m |El valor de vel_th es:"<<vel_th<<std::endl;


    //Read and print MPU
    // read sensor data
    if(rc_mpu_read_accel(&data)<0){
        printf("read accel data failed\n");
    }
    if(rc_mpu_read_gyro(&data)<0){
        printf("read gyro data failed\n");
    }

    std::cout << "Giro X "<<data.gyro[0]<<"deg | Giro y: "<<data.gyro[1]<<"Giro Z: "<<data.gyro[2]<<std::endl;

    //Gyrodometry
    double theta_thereshold=0.125/dt;

    delta_theta_gyro=data.gyro[2]-old_gyroz;

   // if(abs(delta_theta_gyro-delta_th)>theta_thereshold){
   //       double delta_th=th+delta_theta_gyro*dt;
   // }else{
   //       double delta_th=th+delta_th*dt;
   // }

    old_gyrox=data.gyro[0];
    old_gyroy=data.gyro[1];
    old_gyroz=data.gyro[2];
    x = x + delta_x;
    y = y + delta_y;
    th = th + delta_th;
 
    // Aqui ya calculo todo ----------------<<<<<<<<<<<<<<
    //
    /**
    double velocity_left = (vx - vth * g_wheelbase / 2.0);
    double velocity_right = (vx + vth * g_wheelbase / 2.0);

    //Motor duty command send
    // calcaulate duty cycle form velocity and duty factor
    double duty_left = g_duty_factor * velocity_left;
    // multiplicar 1.175 el duty right
    double duty_right = g_duty_factor * velocity_right;

    ROS_INFO("set LEFT motor: velocity:%f duty:%f RIGHT motor: velocity:%f duty:%f", velocity_left, duty_left,
           velocity_right, duty_right);

    rc_motor_set(g_left_motor, duty_left);
    rc_motor_set(g_right_motor, duty_right);
    g_driving = 1;
    **/
    // Hasta aca comienza  publicar ----------------<<<<<<<<<<<<<<<<<< 


    //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
    //:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
    //CONTROL DE VELOCIDAD lineal  DE MOTORES
    //
    //
    
    double allowed_error=0.02;
    double error_lineal = vx - vel_x;
    double error_angular = vth - vel_th;

    double old_integral_lineal;
    double old_integral_angular;
    double integral_lineal = old_integral_lineal + error_lineal;
    double integral_angular = old_integral_angular + error_angular;

    double kp_l = 3;
    double ki_l = 0;


    double kp_a = 2;
    double ki_a = 0;


    double controlPID_lineal = (vx+(kp_l*error_lineal) + (ki_l*integral_lineal)); 
    double controlPID_angular = vth + (kp_a*error_angular) + (ki_a*integral_angular);

    if (error_lineal <= allowed_error && error_angular <= allowed_error ){
	
            double velocity_left = (vx -  vth * g_wheelbase / 2.0);
	    double velocity_right = (vx + vth * g_wheelbase / 2.0);
	
	    //Motor duty command send
	    // calcaulate duty cycle form velocity and duty factor
	    double duty_left = g_duty_factor * velocity_left*-1;
	    // multiplicar 1.175 el duty right
	    double duty_right = g_duty_factor * velocity_right;

	    ROS_INFO("set LEFT motor: velocity:%f duty:%f RIGHT motor: velocity:%f duty:%f", velocity_left, duty_left,
           velocity_right, duty_right);

	    rc_motor_set(g_left_motor, duty_left);
	    rc_motor_set(g_right_motor, duty_right);
	    g_driving = 1;


    }
    else{
	    double velocity_left =((controlPID_lineal - controlPID_angular) * g_wheelbase / 2.0);
            double velocity_right = ((controlPID_lineal + controlPID_angular) * g_wheelbase / 2.0);
        
            //Motor duty command send
            // calcaulate duty cycle form velocity and duty factor
            double duty_left = g_duty_factor * velocity_left*-1;
            // multiplicar 1.175 el duty right
            double duty_right = g_duty_factor * velocity_right;

            ROS_INFO("set LEFT motor: velocity:%f duty:%f RIGHT motor: velocity:%f duty:%f", velocity_left, duty_left,
           velocity_right, duty_right);

            rc_motor_set(g_left_motor, duty_left);
            rc_motor_set(g_right_motor, duty_right);
            g_driving = 1;

    }
    
    old_integral_lineal = integral_lineal;
    old_integral_angular = integral_angular;





    //
    //
    //:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
    //::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::


    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vel_x;
    odom.twist.twist.linear.y = vel_y;
    odom.twist.twist.angular.z = vel_th;

    // publish the message
    odom_pub.publish(odom);

    last_time = current_time;

    r.sleep();
  }

  // %EndTag(SPIN)%

  // close hardware
  ROS_INFO("Calling rc_motor_cleanup()");
  rc_motor_cleanup();
  ROS_INFO("Calling rc_encoder_cleanup()");
  rc_encoder_cleanup();
  rc_mpu_power_off();
  return 0;
}


