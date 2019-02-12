#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include <iostream>

struct JitterFilter {
public:
    explicit JitterFilter(ros::NodeHandle &node)
    : last_stamp_{ 0, 0 },
      publisher_{ node.advertise<sensor_msgs::Imu>("imu/data_jitter_filtered", 10) }
    { }

    void filter(const sensor_msgs::Imu::ConstPtr &imu_ptr) {
        //const auto now = ros::Time::now();
        //const auto stamp = imu_ptr->header.stamp;

        sensor_msgs::Imu new_data = *imu_ptr;
        static sensor_msgs::Imu prev_data = *imu_ptr;

        const float difference = 0.1;
	//ROS_INFO("in filter");

	/*
        if (stamp <= last_stamp_ || stamp > now) {
	    ROS_INFO("stuff2");
            return;
        }
	*/

        bool lin_accel_ok = dataFilter(new_data.linear_acceleration, prev_data.linear_acceleration, difference);
        bool ang_vel_ok = dataFilter(new_data.angular_velocity, prev_data.angular_velocity, difference);

        //last_stamp_ = stamp;

        if (lin_accel_ok && ang_vel_ok)
	{
        	publisher_.publish(new_data);
	}
	else
	{
		std::cout << "Jitter Filter rejected incoming IMU message" << std::endl;
	}
	
        prev_data = new_data;
    }

private:
    ros::Time last_stamp_;
    ros::Publisher publisher_;
    bool dataFilter(const auto &data, const auto &prev_data, const float difference)
    {
	//ROS_INFO("stuff");
    	if (abs(data.x - prev_data.x) > difference)
    	{
    		return false;
    	}

    	if (abs(data.y - prev_data.y) > difference)
    	{
    		return false;
    	}

    	if (abs(data.z - prev_data.z) > difference)
    	{
    		return false;
    	}

    	return true;
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "imu_jitter_filter_node");
    ros::NodeHandle node;
	ROS_INFO("Jitter Filter launched");

    JitterFilter filter{ node };
    const auto subscriber =
        node.subscribe<sensor_msgs::Imu>("imu/data_filtered", 10,
                                         &JitterFilter::filter, &filter);

    ros::spin();
}

