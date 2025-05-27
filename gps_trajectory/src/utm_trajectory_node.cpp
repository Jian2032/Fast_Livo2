#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <serial_port/GPS.h>
#include <GeographicLib/UTMUPS.hpp>

class GpsToPath {
public:
  GpsToPath() {
    // 初始化发布器
    original_path_pub_ = nh_.advertise<nav_msgs::Path>("/original_path", 10);
    corrected_path_pub_ = nh_.advertise<nav_msgs::Path>("/corrected_path", 10);

    // 初始化路径消息
    original_path_.header.frame_id = "map";
    corrected_path_.header.frame_id = "map";

    // 订阅GPS数据
    sub_ = nh_.subscribe("/gps_rec/gps_data", 10, &GpsToPath::callback, this);
  }

private:
  void callback(const serial_port::GPS::ConstPtr& msg) {
    // 转换原始经纬度
    double lat = msg->latitude / 1e7;
    double lon = msg->longitude / 1e7;
    double corrected_lat = msg->corrected_lat / 1e7;
    double corrected_lon = msg->corrected_lon / 1e7;

    // UTM转换
    try {
      // 原始坐标转换
      int zone;
      bool northp;
      double original_x, original_y;
      GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, original_x, original_y);

      // 修正坐标转换
      double corrected_x, corrected_y;
      GeographicLib::UTMUPS::Forward(corrected_lat, corrected_lon, 
                                   zone, northp, corrected_x, corrected_y);

      // 设置原点（第一个点）
      if (!origin_initialized_) {
        origin_easting_ = original_x;
        origin_northing_ = original_y;
        origin_initialized_ = true;
        ROS_INFO("Origin set to: %.3f, %.3f", origin_easting_, origin_northing_);
      }

      // 创建PoseStamped
      geometry_msgs::PoseStamped original_pose, corrected_pose;
      original_pose.header.stamp = ros::Time::now();
      original_pose.header.frame_id = "map";
      original_pose.pose.position.x = original_x - origin_easting_;
      original_pose.pose.position.y = original_y - origin_northing_;
      original_pose.pose.orientation.w = 1.0;

      corrected_pose.header.stamp = ros::Time::now();
      corrected_pose.header.frame_id = "map";
      corrected_pose.pose.position.x = corrected_x - origin_easting_;
      corrected_pose.pose.position.y = corrected_y - origin_northing_;
      corrected_pose.pose.orientation.w = 1.0;

      // 更新路径
      original_path_.poses.push_back(original_pose);
      corrected_path_.poses.push_back(corrected_pose);
      original_path_.header.stamp = ros::Time::now();
      corrected_path_.header.stamp = ros::Time::now();

      // 发布路径
      original_path_pub_.publish(original_path_);
      corrected_path_pub_.publish(corrected_path_);

    } catch (const std::exception& e) {
      ROS_ERROR("UTM转换失败: %s", e.what());
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher original_path_pub_;
  ros::Publisher corrected_path_pub_;
  ros::Subscriber sub_;

  nav_msgs::Path original_path_;
  nav_msgs::Path corrected_path_;

  double origin_easting_ = 0;
  double origin_northing_ = 0;
  bool origin_initialized_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_to_path");
  GpsToPath node;
  ros::spin();
  return 0;
}