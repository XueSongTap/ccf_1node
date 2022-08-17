#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "serial/serial.h"
#include <string>
#include <geometry_msgs/Vector3Stamped.h>

//imu message 没有euler data ，使用 vector3stamped, 带时间， x,y, z 三个数据
/*

# This represents a Vector3 with reference coordinate frame and timestamp
Header header
Vector3 vector

*/


ros::Publisher IMU_pub;
ros::Publisher Euler_pub;
/*
数据类型 数据 ID 长度 内容
加速度 0x10 12 DATA1 – DATA12
角速度 0x20 12 DATA1 – DATA12
欧拉角 0x40 12 DATA1 – DATA12
四元数 0x41 16 DATA1 – DATA16
*/

enum YesenseMsg {
    YESENSE_ACCEL = 0x10,
    YESENSE_ANGLUAR = 0x20,
    YESENSE_EULER = 0x40,
    YESENSE_QUATERNION = 0x41,
};


#pragma pack(1)

struct yesense_accel{
    int32_t ax_e6;
    int32_t ay_e6;
    int32_t az_e6;
};

struct yesense_angular {
    int32_t wx_e6;
    int32_t wy_e6;
    int32_t wz_e6;
};

//定义欧拉角pitch roll yaw
struct yesense_euler {
    int32_t pitch_e6;
    int32_t roll_e6;
    int32_t yaw_e6;
};
//定义传感器四元数组
struct yesense_quaternion {
    int32_t q_e6[4];
};

#pragma pack()


struct yesense_msg {
    //tid帧头？
    unsigned tid;
    //struct yesense_temperature *temperature;
    struct yesense_accel *accel;
    struct yesense_angular *angular;
    struct yesense_euler *euler;
    struct yesense_quaternion *quaternion;
    //struct yesense_utctime *utc;

};


static void yesense_checksum(const unsigned char *buf, size_t sz, unsigned char *ck1, unsigned char* ck2){
    unsigned char c1 = 0, c2 = 0;

    for (int i = 0; i < sz; i ++){
        c1 += buf[i];

        c2 += c1;
    }

    *ck1 = c1;
    *ck2 = c2;
}


//作为输出使用检查数据
double yaww = 0.0;
static void yesense_msg_parse(const unsigned char *buf, size_t sz, struct yesense_msg* msg){
    msg -> accel = NULL;
    msg -> angular = NULL;
    msg->euler = NULL;
    msg -> quaternion = NULL;
    static struct yesense_accel       y_accel;
    static struct yesense_angular     y_angular;
    static struct yesense_euler       y_euler;
    static struct yesense_quaternion  y_quaternion;

    ROS_INFO("TID=%u, PAYLOAD=%u", msg->tid, sz);

    int idx = 0;

    while (idx < sz){
        int remain = sz- idx;

        if (remain < 2){
            ROS_ERROR("unexpected end of message");
            break;
        }

        int id = buf[idx];

        int len = buf[idx + 1];
        const unsigned char *p = &buf[idx + 2];

        if (remain < (2 + len)){
            ROS_ERROR("unexpected end of message");

            break;
        }

        switch(id){
            case YESENSE_ACCEL:
                assert(len == sizeof(y_accel) && "bad accel msg size");
                memcpy(&y_accel, p, len);
                msg->accel = &y_accel;
                break;
            case YESENSE_ANGLUAR:
                assert(len == sizeof(y_angular) && "bad angular msg size");
                memcpy(&y_angular, p, len);
                msg->angular = &y_angular;
                break;
            case YESENSE_EULER:
                assert(len == sizeof(y_euler) && "bad euler msg size");
                memcpy(&y_euler, p, len);
                msg->euler = &y_euler;
                break;
            case YESENSE_QUATERNION:
                assert(len == sizeof(y_quaternion) && "bad quaternion msg size");
                memcpy(&y_quaternion, p, len);
                msg->quaternion = &y_quaternion;
                break;
            default:
                ROS_INFO("unsupported msg id=0x%02X len=%d", id, len);
                break;
        }
        
        idx += len + 2;
    }
}

size_t yesense_process(const unsigned char * buf, size_t len){
    if (len < 7) return len;

    if (buf[0] != 0x59 || buf[1] != 0x53){
        return len -1;
    }

    unsigned mlen = buf[4];


    if (len < (7 + mlen)){
        return len;
    }

    unsigned char c1, c2;

    yesense_checksum(buf + 2, 3 + mlen, &c1, &c2);

    if (c1 != buf[5 + mlen] || c2 != buf[6 + mlen]){
        ROS_ERROR("checksum failed, expect %02x, %02x actual %02x, %02x", c1, c2, buf[5 + mlen], buf[6 + mlen]);
        return len - (7 + mlen);
    }
    //<< 8 二进制，移8位
    uint16_t tid = buf[2] + (buf[3] << 8);


    struct yesense_msg y_msg = {0};

    y_msg.tid = tid;
    //buf数据处理后存入struct y_msg
    yesense_msg_parse(buf + 5, mlen,  &y_msg);
    
    /*
    int32_t pitch_e6;
    int32_t roll_e6;
    int32_t yaw_e6;
    */
    ROS_INFO("pitch_e6: %d, roll_e6: %d, yaw_e6: %d\n", y_msg.euler->pitch_e6, y_msg.euler->roll_e6, y_msg.euler->yaw_e6 );
    ros::Time imuT;

    imuT = ros::Time::now();

    sensor_msgs::Imu imumsg;
    geometry_msgs::Vector3Stamped eulermsg;

    imumsg.header.frame_id = "imu";
    eulermsg.header.frame_id = "imu";

    imumsg.header.stamp = imuT;
    eulermsg.header.stamp = imuT;
    

    imumsg.linear_acceleration.x = (double)y_msg.accel->ax_e6*1e-6;
    imumsg.linear_acceleration.y = (double)y_msg.accel->ay_e6*1e-6;
    imumsg.linear_acceleration.z = (double)y_msg.accel->az_e6*1e-6;



    imumsg.angular_velocity.x = (double)y_msg.angular->wx_e6*1e-6*M_PI/180;
    imumsg.angular_velocity.y = (double)y_msg.angular->wy_e6*1e-6*M_PI/180;
    imumsg.angular_velocity.z = (double)y_msg.angular->wz_e6*1e-6*M_PI/180;

    imumsg.orientation.w = (double)y_msg.quaternion->q_e6[0]*1e-6;
    imumsg.orientation.x = (double)y_msg.quaternion->q_e6[1]*1e-6;
    imumsg.orientation.y = (double)y_msg.quaternion->q_e6[2]*1e-6;
    imumsg.orientation.z = (double)y_msg.quaternion->q_e6[3]*1e-6;
    /*
    int32_t pitch_e6;
    int32_t roll_e6;
    int32_t yaw_e6;
    */
    eulermsg.vector.x = (double)y_msg.euler->pitch_e6 * 1e-6;
    eulermsg.vector.y = (double)y_msg.euler->roll_e6 * 1e-6;
    eulermsg.vector.z = (double)y_msg.euler->yaw_e6 * 1e-6;


    IMU_pub.publish(imumsg);
    Euler_pub.publish(eulermsg);
    yaww += (imumsg.angular_velocity.z - 1.1505821405593758e-05)*0.005;
    std::cout<<"yaw "<<yaww<<std::endl;
    std::cout<<"["<<imumsg.header.stamp<<"] "<< "angle: "<<y_msg.euler->pitch_e6*1e-6<<" "<<y_msg.euler->roll_e6*1e-6<<" "<<y_msg.euler->yaw_e6*1e-6<<std::endl;
    
    
    return len - (7 + mlen);
}


int main(int argc, char ** argv){

    unsigned char buf[4096];

    size_t len = 0;

    ros::init(argc, argv, "yesense_driver");


    ros::NodeHandle n;

    //n_private 专门用于读取 数据
    ros::NodeHandle n_private("~");

    int baud_rate = 460800;

    std::string device = "/dev/ttyUSB0";
    std::string imu_topic = "/imu/data_raw";
    std::string euler_topic = "/imu/euler_raw";
    IMU_pub = n.advertise<sensor_msgs::Imu>(imu_topic, 100);
    Euler_pub = n.advertise<geometry_msgs::Vector3Stamped>(euler_topic, 100);
    n_private.param<int>("baud_rate", baud_rate, 460800);

    n_private.param<std::string>("device", device, "/dev/ttyUSB0");
    n_private.param<std::string>("imu_topic", imu_topic, "/imu/data_raw");

    BAUDRATE br = SERIAL_B460800;
    if (serial_num2baudrate(baud_rate, &br) != 0) {
        ROS_ERROR("baudrate '%d' inva   lid", baud_rate);
        return 1;
    }
    SERIAL fd = serial_open(device.c_str(), br);
    if (fd == INVALID_SERIAL) {
        ROS_ERROR("cwnnot open device=%s", device.c_str());
        return 1;
    }   



    // ros::Rate loop_rate(200);

    while (ros::ok()){


        ROS_INFO("serial_read_start");

        int rv = serial_read(fd, buf + len, sizeof(buf) - len);
        if (rv == -1){
            ROS_ERROR("read from dev error");
            break;
        }

        len += rv;

        ROS_INFO("len: %d", len);
        ROS_INFO("sizeof(buf) : %d", sizeof(buf));

        // buf[len == sizeof(buf) ? len - 1 : len] = 0; // 截断，保证 ascii 解析

        size_t newlen = yesense_process(buf, len);

        if (newlen != len){
            memmove(buf,  buf + len - newlen, newlen);
            len = newlen;
        }

        ros::spinOnce();

        // loop_rate.sleep();
    }
}