#include <ros/ros.h>
#include <serial/serial.h>
#include <serial_port/GPS.h>

// 使用您提供的CRC16校验函数
#include "serial_port/crc.h"  // 确保头文件路径正确

CRC crc;

class GpsSerialNode
{
public:
    GpsSerialNode() : nh_("~") {
        // 初始化发布者
        pub_ = nh_.advertise<serial_port::GPS>("gps_data", 10);

        // 配置串口参数
        std::string port;
        nh_.param<std::string>("serial_port", port, "/dev/ttyUSB0");
        
        try {
            ser_.setPort(port);
            ser_.setBaudrate(115200);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(timeout);
            ser_.open();
            ROS_INFO("Serial port opened successfully");
        } catch (const serial::IOException& e) {
            ROS_ERROR_STREAM("Failed to open serial port: " << e.what());
            ros::shutdown();
        }
    }

    void run() {
        std::vector<uint8_t> buffer;

        while (ros::ok()) {
            // 读取串口数据
            if (ser_.available()) {
                std::vector<uint8_t> raw_data;
                ser_.read(raw_data, ser_.available());
                buffer.insert(buffer.end(), raw_data.begin(), raw_data.end());
            }

            // 解析数据帧（总长度 = 22数据字节 + 2CRC字节 = 24字节）
            size_t pos = 0;
            while (pos + 24 <= buffer.size()) {
                // 检查起始标记0xAA55（小端）
                uint16_t start_marker = (buffer[pos+1] << 8) | buffer[pos];
                if (start_marker != 0xAA55) {
                    pos++;
                    continue;
                }

                // 提取数据部分（前22字节）
                uint8_t* frame_data = &buffer[pos];

                // 使用您提供的CRC16校验函数
                uint8_t last_1 = buffer[22];
                uint8_t last_2 = buffer[23];
                crc.Append_CRC16_Check_Sum(frame_data, 24);

                // CRC校验失败处理
                if (last_1 != frame_data[22] || last_2 != frame_data[23]) {
                    ROS_WARN("CRC check failed! Expected: %02X %02X, Got: %02X %02X",
                             last_2, last_1, frame_data[22], frame_data[23]);
                    pos += 24;  // 跳过完整帧
                    continue;
                }

                // --- 解析有效数据 ---
                // 时间字段（1字节各字段）
                uint8_t hour   = buffer[pos+2];
                uint8_t min    = buffer[pos+3];
                uint8_t sec    = buffer[pos+4];
                uint8_t ms100  = buffer[pos+5];


                // 原始坐标（小端）
                uint32_t latitude = (buffer[pos+6]  << 0) | (buffer[pos+7]  << 8) |
                                    (buffer[pos+8]  << 16) | (buffer[pos+9]  << 24);
                uint32_t longitude = (buffer[pos+10] << 0) | (buffer[pos+11] << 8) |
                                     (buffer[pos+12] << 16) | (buffer[pos+13] << 24);

                // 校正坐标（小端）
                uint32_t corrected_lat = (buffer[pos+14] << 0) | (buffer[pos+15] << 8) |
                                         (buffer[pos+16] << 16) | (buffer[pos+17] << 24);
                uint32_t corrected_lon = (buffer[pos+18] << 0) | (buffer[pos+19] << 8) |
                                         (buffer[pos+20] << 16) | (buffer[pos+21] << 24);

                // 填充消息
                serial_port::GPS msg;
                msg.start_marker = start_marker;
                msg.hour = hour;
                msg.min = min;
                msg.sec = sec;
                msg.ms100 = ms100;
                msg.latitude = latitude;
                msg.longitude = longitude;
                msg.corrected_lat = corrected_lat;
                msg.corrected_lon = corrected_lon;
                msg.crc16 = (last_2 << 8) | last_1;  // CRC值
                // 发布消息
                pub_.publish(msg);

                // 日志输出
                ROS_INFO("Time: %02d:%02d:%02d.%02d | Raw(lat=%u, lon=%u) | Corrected(lat=%u, lon=%u)",
                         hour, min, sec, ms100, latitude, longitude, corrected_lat, corrected_lon);

                // 移动到下一帧
                pos += 24;
            }

            // 清除已处理的数据
            if (pos > 0) {
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_rec");
    GpsSerialNode node;
    node.run();
    return 0;
}