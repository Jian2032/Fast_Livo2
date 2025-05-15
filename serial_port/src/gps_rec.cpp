#include <ros/ros.h>
#include <serial/serial.h>
#include <serial_port/GPS.h> // 替换为你的自定义消息包名

class GpsSerialNode
{
public:
    GpsSerialNode() : nh_("~")
    {
        // 初始化发布者
        pub_ = nh_.advertise<serial_port::GPS>("gps_data", 100);

        // 配置串口参数
        std::string port;
        nh_.param<std::string>("serial_port", port, "/dev/ttyUSB0");

        try
        {
            ser_.setPort(port);
            ser_.setBaudrate(115200);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(timeout);
            ser_.open();
            ROS_INFO("Serial port opened successfully");
        }
        catch (const serial::IOException &e)
        {
            ROS_ERROR_STREAM("Failed to open serial port: " << e.what());
            ros::shutdown();
        }
    }

    void run()
    {
        std::vector<uint8_t> buffer;

        while (ros::ok())
        {
            // 读取串口数据
            if (ser_.available())
            {
                std::vector<uint8_t> raw_data;
                ser_.read(raw_data, ser_.available());
                buffer.insert(buffer.end(), raw_data.begin(), raw_data.end());
            }

            // 解析数据帧
            size_t pos = 0;
            while (pos + 14 <= buffer.size())
            { // 每帧14字节
                // 检查起始标记0xAA55 (小端模式)
                uint16_t start_marker = (buffer[pos + 1] << 8) | buffer[pos];
                if (start_marker != 0xAA55)
                {
                    pos++;
                    continue;
                }

                // 提取字段（假设小端字节序）
                uint32_t latitude = (buffer[pos + 2] << 0) |
                                    (buffer[pos + 3] << 8) |
                                    (buffer[pos + 4] << 16) |
                                    (buffer[pos + 5] << 24);

                uint32_t longitude = (buffer[pos + 6] << 0) |
                                     (buffer[pos + 7] << 8) |
                                     (buffer[pos + 8] << 16) |
                                     (buffer[pos + 9] << 24);

                int32_t altitude = (buffer[pos + 10] << 0) |
                                   (buffer[pos + 11] << 8) |
                                   (buffer[pos + 12] << 16) |
                                   (buffer[pos + 13] << 24);

                // 发布消息
                serial_port::GPS msg;
                msg.start_marker = start_marker;
                msg.latitude = latitude;
                msg.longitude = longitude;
                msg.altitude = altitude;
                pub_.publish(msg);
                ROS_INFO("Published GPS data: start_marker=%u, latitude=%u, longitude=%u, altitude=%d",
                         msg.start_marker, msg.latitude, msg.longitude, msg.altitude);

                // 移动到下一帧
                pos += 14;
                break; // 一次处理一帧，避免多帧重叠问题
            }

            // 清除已处理的数据
            if (pos > 0)
            {
                buffer.erase(buffer.begin(), buffer.begin() + pos);
            }

            ros::spinOnce();
        }
    }

private:
    ros::NodeHandle nh_;
    serial::Serial ser_;
    ros::Publisher pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_rec");
    GpsSerialNode node;
    node.run();
    return 0;
}