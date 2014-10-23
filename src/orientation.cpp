#include <cmath>

#include <ros/ros.h>

#include <s8_common_node/Node.h>
#include <sensor_msgs/Imu.h>
#include <s8_msgs/Orientation.h>

#define NODE_NAME           "s8_orientation_node"

#define TOPIC_IMU           "/imu/data"
#define TOPIC_ORIENTATION   "/s8/orientation"

class Orientation : public s8::Node {
    ros::Subscriber imu_subscriber;
    ros::Publisher orientation_publisher;

public:
    Orientation() {
        imu_subscriber = nh.subscribe<sensor_msgs::Imu>(TOPIC_IMU, 1000, &Orientation::imu_callback, this);
        orientation_publisher = nh.advertise<s8_msgs::Orientation>(TOPIC_ORIENTATION, 1000);
    }

private:
    void imu_callback(const sensor_msgs::Imu::ConstPtr & imu_data) {
        double q0 = imu_data->orientation.x;
        double q1 = imu_data->orientation.y;
        double q2 = imu_data->orientation.z;
        double q3 = imu_data->orientation.w;
        
        //double orientation = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
        //double orientation = asin(2 * (q0 * q2 - q3 * q1));
        double orientation = atan2(2 * (q0 * q3 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));

        publish(orientation_to_degrees(orientation));
    }

    void publish(int orientation_degrees) {
        s8_msgs::Orientation orientation;
        orientation.z = orientation_degrees;
        orientation_publisher.publish(orientation);
    }

    int orientation_to_degrees(double orientation) {
        return (orientation / (2 * 3.14)) * 360;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    Orientation orientation;
    ros::spin();
    return 0;
}
