#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

double distance = 2.0;

void obstacle_cb(const std_msgs::Float32::ConstPtr& msg) {
  distance = msg->data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "flight_node");
  ros::NodeHandle nh;

  // 10 Hz 发布，周期 100 ms，远小于后端 500 ms 心跳超时，保证设定点流持续有效
  ros::Rate rate(10.0);

  ros::Publisher g_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone/setpoint", 10);

  // 订阅障碍距离，用于判断何时降落
  ros::Subscriber obstacle_sub = nh.subscribe(
      "/drone/sensor/obstacle_distance", 10, obstacle_cb);

  geometry_msgs::PoseStamped setpoint;
  setpoint.header.frame_id = "map";
  setpoint.pose.position.x = 0.0;
  setpoint.pose.position.y = 0.0;
  setpoint.pose.position.z = 1.5;  // 目标高度 1.5 m，起飞

  ROS_INFO("flight_node ready");
  while (ros::ok()) {

    if (distance < 0.5) {  // 障碍距离小于 0.5 m，目标高度归零，降落
      setpoint.pose.position.z = 0;
    }

    setpoint.header.stamp = ros::Time::now();  // 每帧填当前时间
    g_setpoint_pub.publish(setpoint);  // 持续发布目标点，维持心跳并控制飞机
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("flight_node stopped");
  return 0;
}
