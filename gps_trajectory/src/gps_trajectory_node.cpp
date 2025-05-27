#include <ros/ros.h>
#include <serial_port/GPS.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class LocalTrajectory
{
public:
    LocalTrajectory() : nh_("~"), has_initial_(false)
    {
        // 参数配置
        start_marker_ = 0xAA55;
        nh_.setParam("start_marker", start_marker_);

        // 订阅/发布
        gps_sub_ = nh_.subscribe("/gps_rec/gps_data", 20, &LocalTrajectory::gpsCallback, this);
        original_path_pub_ = nh_.advertise<nav_msgs::Path>("/original_path", 10);
        corrected_path_pub_ = nh_.advertise<nav_msgs::Path>("/corrected_path", 10);
    }

private:
    // 添加所有需要的成员变量（修复2）
    ros::NodeHandle nh_;
    ros::Subscriber gps_sub_;
    ros::Publisher original_path_pub_;
    ros::Publisher corrected_path_pub_;
    nav_msgs::Path original_path_;
    nav_msgs::Path corrected_path_; 

    uint16_t start_marker_; // 必须定义的成员变量
    bool has_initial_;
    double initial_lat_;
    double initial_lon_;

    // 经纬度转局部坐标（平面近似法）
    void convertToLocal(double lat, double lon, double alt,
                        double &x, double &y, double &z)
    {
        if (!has_initial_)
        {
            initial_lat_ = lat; // 保持变量名一致
            initial_lon_ = lon;
            has_initial_ = true;
            x = y = z = 0.0;
            return;
        }

        // 计算经纬度差值（度）
        double delta_lat = lat - initial_lat_;
        double delta_lon = lon - initial_lon_;

        // 转换为米（WGS84近似）
        const double meters_per_degree_lat = 111132.92 - 559.82 * cos(2 * initial_lat_) + 1.175 * cos(4 * initial_lat_);
        const double meters_per_degree_lon = 111412.84 * cos(initial_lat_) - 93.5 * cos(3 * initial_lat_);

        x = delta_lon * meters_per_degree_lon; // 东方向
        y = delta_lat * meters_per_degree_lat; // 北方向
        // z = alt - initial_alt_;                // 天方向
        z = 0;

        ROS_INFO("Converted to local coordinates: x=%f, y=%f, z=%f", x, y, z);
        ROS_INFO("GPS data:                       lat=%f, lon=%f, alt=%f", lat, lon, alt);
    }
    void gpsCallback(const serial_port::GPS::ConstPtr &msg)
    {
        // 校验起始标记
        if (msg->start_marker != start_marker_)
            return;

        // 解码原始数据
        double latitude = static_cast<int32_t>(msg->latitude) / 100000.0;
        double longitude = static_cast<int32_t>(msg->longitude) / 100000.0;
        double latitude_corrected = static_cast<int32_t>(msg->corrected_lat) / 100000.0;
        double longitude_corrected = static_cast<int32_t>(msg->corrected_lon) / 100000.0;

        // 转换到局部坐标
        double x, y, z;
        double x_corrected, y_corrected, z_corrected;
        convertToLocal(latitude, longitude, 0, x, y, z);
        convertToLocal(latitude_corrected, longitude_corrected, 0, x_corrected, y_corrected, z_corrected);

        // 构建位姿消息（ENU坐标系）
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "local_origin";

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.w = 1.0; // 无旋转

        // 更新路径
        original_path_.header = pose.header;
        original_path_.poses.push_back(pose);

        // 构建位姿消息（ENU坐标系）
        geometry_msgs::PoseStamped pose_corrected;
        pose_corrected.header.stamp = ros::Time::now();
        pose_corrected.header.frame_id = "local_origin";

        pose_corrected.pose.position.x = x_corrected;
        pose_corrected.pose.position.y = y_corrected;
        pose_corrected.pose.position.z = z_corrected;
        pose_corrected.pose.orientation.w = 1.0; // 无旋转

        // 更新路径
        corrected_path_.header = pose_corrected.header;
        corrected_path_.poses.push_back(pose_corrected);
        

        original_path_pub_.publish(original_path_);
        corrected_path_pub_.publish(corrected_path_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_trajectory_node");
    LocalTrajectory node;
    ros::spin();
    return 0;
}