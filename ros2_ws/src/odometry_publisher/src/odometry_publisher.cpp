//#include <chrono>
#include <functional>
#include <memory>
//#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define UPDATE_DT 33ms //100ms
#define PUBLISH_DT 33ms //500ms

//#define PI 3.14159265

class OdometryPublisher : public rclcpp::Node
{
    public:
        OdometryPublisher()
        : Node("odometry_publisher")//, enc_r_(0), enc_l_(0), prev_enc_r_(0), prev_enc_l_(0), odom_(std::make_shared<nav_msgs::msg::Odometry>()), tf_odom_(std::make_shared<geometry_msgs::msg::TransformStamped>())
        {
            this->set_parameter(rclcpp::Parameter("use_sim_time", true));
            
            enc_r_sub_ = this->create_subscription<std_msgs::msg::Int32>(
                "/enc_r_values", rclcpp::QoS(10).best_effort(), std::bind(&OdometryPublisher::enc_r_cb, this, _1));
            
            enc_l_sub_ = this->create_subscription<std_msgs::msg::Int32>(
                "/enc_l_values", rclcpp::QoS(10).best_effort(), std::bind(&OdometryPublisher::enc_l_cb, this, _1));
            
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry", 10);

            ////tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            update_odometry_timer_ = this->create_wall_timer(UPDATE_DT, std::bind(&OdometryPublisher::update_odometry, this));
            
            publish_odometry_timer_ = this->create_wall_timer(PUBLISH_DT, std::bind(&OdometryPublisher::publish_odometry, this));
            
            //update_odometry_timer_ = rclcpp::create_timer(this, this->get_clock(), std::chrono::milliseconds(100), std::bind(&OdometryPublisher::update_odometry, this));
            //publish_odometry_timer_ = rclcpp::create_timer(this, this->get_clock(), std::chrono::milliseconds(100), std::bind(&OdometryPublisher::publish_odometry, this));
        }
    private:
        void enc_r_cb(const std_msgs::msg::Int32::SharedPtr msg)
        {
            enc_r_ = msg->data;
            //RCLCPP_INFO(this->get_logger(), "I heard: %d", msg->data);
        }

        void enc_l_cb(const std_msgs::msg::Int32::SharedPtr msg)
        {
            enc_l_ = msg->data;
            //RCLCPP_INFO(this->get_logger(), "I heard: %d", msg->data);
        }

        /* --- ros2_controllers diff_drive_controller --- */
        void integrateRungeKutta2(double linear, double angular)
        {
            const double direction = heading_ + angular * 0.5;

            /// Runge-Kutta 2nd order integration:
            x_ += linear * cos(direction);
            y_ += linear * sin(direction);
            heading_ += angular;
        }

        void integrateExact(double linear, double angular)
        {
            if (fabs(angular) < 1e-6)
            {
                integrateRungeKutta2(linear, angular);
            }
            else
            {
                /// Exact integration (should solve problems when angular is zero):
                const double heading_old = heading_;
                const double r = linear / angular;
                heading_ += angular;
                x_ += r * (sin(heading_) - sin(heading_old));
                y_ += -r * (cos(heading_) - cos(heading_old));
            }
        }
        /* ------------------------------------------------ */

        void update_odometry() 
        {
            //printf("enc_r:%d, prev_enc_r:%d, enc_l:%d prev_enc_l:%d\n", enc_r_, prev_enc_r_, enc_l_, prev_enc_l_);

            //rclcpp::Time now = this->get_clock()->now();
            //rclcpp::Clock ros_clock(RCL_ROS_TIME);
            //rclcpp::Time now = ros_clock.now();
            rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
            float delta_t = (now - last_time).nanoseconds() / 1e9;  // unit: s
            //printf("delta_time: %f\n", delta_t);
            last_time = now;

            //auto dt_ms = UPDATE_DT; //[ms]
            //auto dt_s = dt_ms.count() / 1000.0; //[s]

            float d_right = (enc_r_ - prev_enc_r_) * 2 * M_PI * wheel_radius / ticks_per_revolution;
            float d_left = (enc_l_ - prev_enc_l_) * 2 * M_PI * wheel_radius / ticks_per_revolution;

            prev_enc_r_ = enc_r_;
            prev_enc_l_ = enc_l_;

            float d_center = (d_right + d_left) * 0.5;
            float phi = (d_right - d_left) / base_width;

            integrateExact(d_center, phi);

            /*
            theta_end = theta + phi;
            if (theta_end >= 2.0 * M_PI) theta_end -= 2.0 * M_PI;
            if (theta_end < 0.0) theta_end += 2.0 * M_PI;

            x_end = x + d_center * std::cos(theta);
            y_end = y + d_center * std::sin(theta);
            */

            odom_->header.stamp = now;
            odom_->header.frame_id = "odom";
            odom_->child_frame_id = "base_footprint";
            //printf("frame_id:%s\n", odom_->header.frame_id.c_str());

            odom_->pose.pose.position.x = x_;
            odom_->pose.pose.position.y = y_;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, heading_);
            odom_->pose.pose.orientation.x = q.getX();
            odom_->pose.pose.orientation.y = q.getY();
            odom_->pose.pose.orientation.z = q.getZ();
            odom_->pose.pose.orientation.w = q.getW();

            odom_->twist.twist.linear.x = d_center / delta_t;//dt_s;
            odom_->twist.twist.angular.z = phi / delta_t;//dt_s;
            
            // Send the transformation
            /*
            tf_odom_->header = odom_->header;
            tf_odom_->child_frame_id = odom_->child_frame_id;
            tf_odom_->transform.translation.x = odom_->pose.pose.position.x;
            tf_odom_->transform.translation.y = odom_->pose.pose.position.y;
            tf_odom_->transform.translation.z = odom_->pose.pose.position.z;
            tf_odom_->transform.rotation = odom_->pose.pose.orientation;
            */
            //theta = theta_end;
            //x = x_end;
            //y = y_end;
        }

        void publish_odometry()
        {
            //tf_broadcaster_->sendTransform(*tf_odom_);
            odom_pub_->publish(*odom_);
        }

        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr enc_r_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr enc_l_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr update_odometry_timer_;
        rclcpp::TimerBase::SharedPtr publish_odometry_timer_;

        const float wheel_radius = 0.035; //[m] 70mm/2
        const float base_width = 0.141; //[m] 149mm - 8mm
        //const float ticks_meter = 6548.089; //[count/m] 1440 / (2*pi*0.035)
        const float ticks_per_revolution = 1440.0;

        /*
        float x = 0.0;
        float y = 0.0;
        float theta = 0.0;
        float x_end = 0.0;
        float y_end = 0.0;
        float theta_end = 0.0;
        */
        float x_ = 0.0;
        float y_ = 0.0;
        float heading_ = 0.0;

        int32_t enc_r_ = 0;
        int32_t enc_l_ = 0;
        int32_t prev_enc_r_ = 0;
        int32_t prev_enc_l_ = 0;

        //rclcpp::Time last_time = this->get_clock()->now();
        //rclcpp::Clock ros_clock(rcl_clock_type_t RCL_ROS_TIME);
        rclcpp::Time last_time = rclcpp::Clock(RCL_ROS_TIME).now();        

        nav_msgs::msg::Odometry::SharedPtr odom_ = std::make_shared<nav_msgs::msg::Odometry>();
        //geometry_msgs::msg::TransformStamped::SharedPtr tf_odom_ = std::make_shared<geometry_msgs::msg::TransformStamped>();
};

int main(int argc, char * argv[])
{
    printf("Odometry Publisher Start\n");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}
