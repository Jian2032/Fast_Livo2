#include <ros/ros.h>
#include <serial_port/GPS.h>
#include <nav_msgs/Path.h>
#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>

class UTMTrajectory {
public:
    UTMTrajectory() : nh_("~"), has_initial_(false) {
        // 参数配置
        start_marker_ = 0xAA55;
        nh_.setParam("start_marker", start_marker_);
        
        // 订阅/发布
        gps_sub_ = nh_.subscribe("/gps_rec/gps_data", 10, &UTMTrajectory::gpsCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/utm_path", 10);
    }

private:
    void gpsCallback(const serial_port::GPS::ConstPtr& msg) {
        // 1. 校验起始标记
        if (msg->start_marker != start_marker_) {
            ROS_WARN("Invalid start marker: 0x%04X", msg->start_marker);
            return;
        }

        // 2. 解码原始数据（考虑符号）
        const double lat = static_cast<int32_t>(msg->latitude)  / 100000.0;
        const double lon = static_cast<int32_t>(msg->longitude) / 100000.0;
        const double alt = static_cast<int32_t>(msg->altitude)  / 10000.0;

        // 3. 转换为UTM坐标
        geographic_msgs::GeoPoint geo_point;
        geo_point.latitude = lat;
        geo_point.longitude = lon;
        geo_point.altitude = alt;

        geodesy::UTMPoint utm;
        try {
            geodesy::fromMsg(geo_point, utm);
        } catch (const std::exception& e) {
            ROS_ERROR("UTM conversion failed: %s", e.what());
            return;
        }

        // 4. 设置初始参考点
        if (!has_initial_) {
            initial_utm_ = utm;
            has_initial_ = true;
            ROS_INFO("Initial UTM Zone: %d%c", initial_utm_.zone, initial_utm_.band);
        }

        // 5. 检查UTM区域一致性
        if (utm.zone != initial_utm_.zone || utm.band != initial_utm_.band) {
            ROS_WARN_THROTTLE(60, "UTM zone changed: %d%c -> %d%c", 
                            initial_utm_.zone, initial_utm_.band,
                            utm.zone, utm.band);
        }

        // 6. 计算局部坐标（相对于初始点）
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "utm_origin";
        
        pose.pose.position.x = utm.easting - initial_utm_.easting;  // 东方向
        pose.pose.position.y = utm.northing - initial_utm_.northing; // 北方向 
        pose.pose.position.z = utm.altitude - initial_utm_.altitude;

        // 7. 更新路径
        path_.header = pose.header;
        path_.poses.push_back(pose);

        // 保持最近1000个点
        if (path_.poses.size() > 1000) {
            path_.poses.erase(path_.poses.begin());
        }

        path_pub_.publish(path_);
    }

    ros::NodeHandle nh_;
    ros::Subscriber gps_sub_;
    ros::Publisher path_pub_;
    nav_msgs::Path path_;
    
    // UTM相关参数
    bool has_initial_;
    geodesy::UTMPoint initial_utm_;
    uint16_t start_marker_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "utm_trajectory_node");
    UTMTrajectory node;
    ros::spin();
    return 0;
}