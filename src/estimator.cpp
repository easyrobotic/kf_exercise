#include "estimator.h"

EstimatorNode::EstimatorNode() {

  ros::NodeHandle nh;

  /* Read parameters from the parameter server */
  ROS_INFO ("Reading Parameters from the parameter server"); 

  readPars(nh);

  ROS_INFO ("Creating data structures for the Kalman Filter"); 

   /* BEGIN: Instantiate here the matrices and vectors of the Kalman filter*/

	// check documentation

   
  /* END: Instantiate here the matrices and vectors of the Kalman filter*/
  
  /* BEGIN: Set the constants C R*/
  ROS_INFO ("Set constants");


  gravity_w = tf::Vector3(0.,0.,-9.8);

  /* END: Set the constants*/

    /* BEGIN: Set the initial conditions: P_hat0*/
  ROS_INFO ("Set the initial conditions "); 

 
  /* END: Set the initial conditions*/

  ROS_INFO ("Creating subscribers, publisher and timer"); 

  // subscriber
  pose_sub_             = nh.subscribe("/gps", 1, &EstimatorNode::GPSCallback, this);
  imu_sub_              = nh.subscribe("/imu", 1, &EstimatorNode::ImuCallback, this);
  
  // publisher
  odometry_pub_  = nh.advertise<nav_msgs::Odometry>("/odom_filtered", 1);

  // timer
  time_reference = ros::WallTime::now(); 

}

EstimatorNode::~EstimatorNode() { }

void EstimatorNode::predict ()
{
  //publish your data
  //ROS_INFO("Publishing ...");

  ros::WallTime time_reference_new = ros::WallTime::now();
  double dT = (time_reference_new-time_reference).toSec();

  /* BEGIN: Compute A, B and Q for dt  and run prediction step computing x-, P-, z- */


  /* END: Compute A, B and Q for dt and end of running prediction step computing x-, P-, z-*/

  time_reference = time_reference_new;

	// Print all the debugging info you may need
  /*
  ROS_INFO ("\n\n");
  ROS_INFO ("Debugging prediction step (dt= %0.4fsec)", dT);
  ROS_INFO("Pos estimation %f %f %f", x_hat(0),x_hat(1),x_hat(2));
  ROS_INFO("Vel estimation %f %f %f", x_hat(3),x_hat(4),x_hat(5));
  */
}

void EstimatorNode::update  ()
{
    /* BEGIN: Compute L, x+ and P+ */

    
    /* END: Compute L, x+ and P+ */


	// Print all the debugging info you may need
  /*
	ROS_INFO ("\n\n");
	ROS_INFO ("Debugging update step");
  ROS_INFO("Pos estimation %f %f %f", x_hat(0),x_hat(1),x_hat(2));
  ROS_INFO("Vel estimation %f %f %f", x_hat(3),x_hat(4),x_hat(5));
  */
}

void EstimatorNode::ImuCallback(
    const sensor_msgs::ImuConstPtr& imu_msg) {

  ROS_INFO_ONCE("Estimator got first IMU message.");

  incomingImuMsg_ = *imu_msg; 

  if (calibrating)
  {
    calib_imu_att_q_buffer.push_back(tf::Quaternion(imu_msg->orientation.x,imu_msg->orientation.y,imu_msg->orientation.z,imu_msg->orientation.w));
    calib_imu_ang_vel_buffer.push_back(tf::Vector3(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z));
    calib_imu_accel_buffer.push_back(tf::Vector3(imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z));

    time_reference = ros::WallTime::now(); 
  }else
  {
    msgOdometry_.header.stamp = imu_msg->header.stamp;
     
    /* BEGIN: Process the acceleration: remove bias, rotate and remove gravity*/
    tf::Vector3     imu_accel ( imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
    tf::Quaternion  imu_att_q(imu_msg->orientation.x,imu_msg->orientation.y,imu_msg->orientation.z,imu_msg->orientation.w);

    imu_att_q = pose2imu_rotation*imu_att_q;
    imu_accel = gravity_w + tf::quatRotate(imu_att_q, imu_accel-imu_accel_bias);


    /* END: Process the acceleration: remove bias, rotate and remove gravity*/
    
	  u << imu_accel_bias[0],imu_accel_bias[1],imu_accel_bias[2];
	
    predict();

    publishPose(); 

  }
}

void EstimatorNode::GPSCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {

  ROS_INFO_ONCE("Estimator got first pose message.");

  incomingPoseMsg_ = *pose_msg; 

  if (calibrating)
  {
    calib_pose_sensor_att_buffer.push_back(tf::Quaternion(pose_msg->pose.pose.orientation.x,
                                                          pose_msg->pose.pose.orientation.y,
                                                          pose_msg->pose.pose.orientation.z,
                                                          pose_msg->pose.pose.orientation.w));
    calib_pose_sensor_pos_buffer.push_back(tf::Vector3( pose_msg->pose.pose.position.x,
                                                        pose_msg->pose.pose.position.y,
                                                        pose_msg->pose.pose.position.z));
  }else
  {
    msgOdometry_.header.stamp = pose_msg->header.stamp;

    /* BEGIN: Generate the measurement y and call update*/
    y << pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z;
    
    predict();
    
    update();
     
    /* END: Generate the measurement y and call update*/

    publishPose();  
  }

  
}

void EstimatorNode::publishPose()
{
  //publish your data
  //ROS_INFO("Publishing ...");
  msgOdometry_.header.frame_id = "world";
  msgOdometry_.child_frame_id = "imu";

  // publish also as pose with covariance 
  msgOdometry_.pose.pose.position.x = x_hat[0];
  msgOdometry_.pose.pose.position.y = x_hat[1];
  msgOdometry_.pose.pose.position.z = x_hat[2];

  msgOdometry_.twist.twist.linear.x = x_hat[3];
  msgOdometry_.twist.twist.linear.y = x_hat[4];
  msgOdometry_.twist.twist.linear.z = x_hat[5];

  // Take the orientation directly from IMU since we don't estimate it 
  msgOdometry_.pose.pose.orientation = incomingImuMsg_.orientation;

  // fill in the values corresponding to position in the covariance

  for (int ii = 0; ii <3; ii++){

    for (int jj = 0; jj<3; jj++){
      msgOdometry_.pose.covariance[ii*6+jj] = P_hat(ii,jj);
      msgOdometry_.twist.covariance[ii*6+jj] = P_hat(ii+3,jj+3);
    }
    
  }

  odometry_pub_.publish(msgOdometry_);

}

void  EstimatorNode::readPars (ros::NodeHandle& nh) { 
    nh.getParam("estimation/gps_covariance", sigma_sqr_gps);
    nh.getParam("estimation/process_covariance", sigma_sqr_process);
    nh.getParam("estimation/initial_error_covariance", sigma_sqr_P0);
}

void  EstimatorNode::startCalibration () { 
  calibrating = true; 
  ROS_INFO_ONCE("Calibration initiated.");
}

void  EstimatorNode::endCalibration ()   { 

  imu_att_q_bias   = averageQuaternion(calib_imu_att_q_buffer);
  imu_ang_vel_bias = averageVector3(calib_imu_ang_vel_buffer);
  imu_accel_bias   = averageVector3(calib_imu_accel_buffer);
  pose_sensor_pos_offset = averageVector3(calib_pose_sensor_pos_buffer);
  pose_sensor_att_bias   = averageQuaternion(calib_pose_sensor_att_buffer);

  pose2imu_rotation = pose_sensor_att_bias*imu_att_q_bias.inverse();
  imu_accel_bias = -imu_accel_bias - tf::quatRotate( (pose2imu_rotation*imu_att_q_bias).inverse(), gravity_w);

  calibrating = false;
  ROS_INFO_ONCE("Calibration ended. Summary: ");
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  ROS_INFO("IMU samples %d Pose samples %d", (int)calib_imu_att_q_buffer.size(), (int)calib_pose_sensor_pos_buffer.size());
  double roll, pitch, yaw;
  tf::Matrix3x3(imu_att_q_bias).getRPY(roll, pitch, yaw);
  ROS_INFO("IMU RPY bias %f %f %f", roll, pitch, yaw);
  ROS_INFO("IMU Ang.Vel bias %f %f %f", imu_ang_vel_bias.x(), imu_ang_vel_bias.y(), imu_ang_vel_bias.z());
  ROS_INFO("IMU Accel. bias %f %f %f", imu_accel_bias.x(), imu_accel_bias.y(), imu_accel_bias.z());
  ROS_INFO("Pose Sensor pose bias %f %f %f", pose_sensor_pos_offset.x(), pose_sensor_pos_offset.y(), pose_sensor_pos_offset.z());
  tf::Matrix3x3(pose_sensor_att_bias).getRPY(roll, pitch, yaw);
  ROS_INFO("Pose Sensor RPY bias %f %f %f", roll, pitch, yaw);
  tf::Matrix3x3(pose2imu_rotation).getRPY(roll, pitch, yaw);
  ROS_INFO("Offset Pose to IMU RPY %f %f %f", roll, pitch, yaw);
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 

  // Set initial state values
  x_hat << pose_sensor_pos_offset[0], pose_sensor_pos_offset[1], pose_sensor_pos_offset[2], 0.0, 0.0, 0.0;

  // free memory
  calib_imu_att_q_buffer.clear(); 
  calib_imu_ang_vel_buffer.clear(); 
  calib_imu_accel_buffer.clear(); 
  calib_pose_sensor_att_buffer.clear(); 
  calib_pose_sensor_pos_buffer.clear(); 
}

tf::Quaternion EstimatorNode::averageQuaternion(std::vector<tf::Quaternion> vec) // It is hacky to do it in RPY
{  
  if (vec.size() == 0) 
    return tf::Quaternion();

  double roll, pitch, yaw;
  double calib_roll, calib_pitch, calib_yaw;
  calib_roll = calib_pitch = calib_yaw = 0.0;
  for(int i = 0; i < vec.size(); i++) 
  {
    tf::Matrix3x3(vec[i]).getRPY(roll, pitch, yaw);
    calib_roll += roll;
    calib_pitch += pitch;
    calib_yaw += yaw;
  }
  calib_roll  = calib_roll / (double)vec.size();
  calib_pitch = calib_pitch / (double)vec.size();
  calib_yaw   = calib_yaw / (double)vec.size();

  return tf::createQuaternionFromRPY(calib_roll, calib_pitch, calib_yaw);
}

tf::Vector3 EstimatorNode::averageVector3(std::vector<tf::Vector3> vec)
{
  if (vec.size() == 0) 
    return tf::Vector3();

  tf::Vector3 res(0.0, 0.0, 0.0);
  for(int i = 0; i < vec.size(); i++) 
  {
    res += vec[i];
  }
  res /= vec.size();

  return res;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "estimator");

  ROS_INFO ("Starting estimator node"); 

  EstimatorNode estimator_node;

  ROS_INFO("Waiting for simulation to start up...");  
  while (ros::WallTime::now().toSec() < 0.2)
  {
    ROS_INFO("Waiting for simulation to start up...");  
    ros::spinOnce();
    ros::Duration(0.01).sleep();     
  }

  //give time to start up the simulation
  ros::Duration(0.2).sleep();     

  // Initialize your filter / controller.
  ROS_INFO("Calibrating offsets for 2 secs...");
  estimator_node.startCalibration(); 
  // 2 secs for init the filters
  ros::WallTime time_reference = ros::WallTime::now();
  while ((ros::WallTime::now()-time_reference).toSec() < 2.0)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();     
  }

  estimator_node.endCalibration(); 

  // let it go .. 
  ros::spin();

  return 0;
}
