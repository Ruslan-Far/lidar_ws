#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <car_msgs/MotorsControl.h>
#include <stdint.h>
#include <car_hackathon/CarHackathonConfig.h>
#include <dynamic_reconfigure/server.h>

float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

const float dt = 0.026; // в секундах

float sumErr = 0.0;
float prevErr = 0.0;

float limitPxl = 0.0;

void image_callback(const sensor_msgs::Image::ConstPtr& msg);
void motors(int16_t left, int16_t right);
int16_t truncate(int16_t pwm);

void reconfigureCB(car_hackathon::CarHackathonConfig &config, uint32_t level)
{
    ROS_INFO("Config changed");
	Kp = config.Kp;
	Ki = config.Ki;
	Kd = config.Kd;
	limitPxl = config.limitPxl;
}

const uint16_t MAX_PWM = 255;
ros::Publisher pub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_follower");
    ros::NodeHandle nh;

	std::shared_ptr<dynamic_reconfigure::Server<car_hackathon::CarHackathonConfig>> dsrv_ = std::make_shared<
    						dynamic_reconfigure::Server<car_hackathon::CarHackathonConfig>>(ros::NodeHandle("/car_hackathon/config"));

	dynamic_reconfigure::Server<car_hackathon::CarHackathonConfig>::CallbackType cb
		= boost::bind(&reconfigureCB, _1, _2);
	dsrv_->setCallback(cb);

    auto sub = nh.subscribe("/car_gazebo/camera1/image_raw", 5, image_callback);
    pub = nh.advertise<car_msgs::MotorsControl>("/motors_commands", 10);

    ros::spin();
    return 0;
}

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{        
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    cv::Mat hsvImage;
    cv:cvtColor(image, hsvImage, CV_BGR2HSV);

    cv::Mat mask;
    cv::Scalar lower(0, 190, 0);
    cv::Scalar upper(179, 255, 255);
    cv::inRange(hsvImage, lower, upper, mask);

    int width = mask.cols;
    int height = mask.rows;

    int search_top = 3 * height / 4;
    int search_bottom = search_top + 20;

	for (int y = 0; y < height - 2; y++) {
		if (y < search_top || y > search_bottom) {
			for (int x = 0; x < width; x++) {
				mask.at<cv::Vec3b>(y, x)[0] = 0;
				mask.at<cv::Vec3b>(y, x)[1] = 0;
				mask.at<cv::Vec3b>(y, x)[2] = 0;
			}
		}
	}

    cv::Moments M = cv::moments(mask);
    if (M.m00 > 0 && M.m00 < limitPxl) {
		// if (M.m00 > limitPxl) {
		// 	for (int y = search_top; y <= search_bottom; y++) {
		// 		for (int x = 0; x < width / 2.0 - 50.0; x++) {
		// 			mask.at<cv::Vec3b>(y, x)[0] = 0;
		// 			mask.at<cv::Vec3b>(y, x)[1] = 0;
		// 			mask.at<cv::Vec3b>(y, x)[2] = 0;
		// 		}
		// 	}
		// 	for (int y = search_top; y <= search_bottom; y++) {
		// 		for (int x = width / 2.0 + 50.0; x < width; x++) {
		// 			mask.at<cv::Vec3b>(y, x)[0] = 0;
		// 			mask.at<cv::Vec3b>(y, x)[1] = 0;
		// 			mask.at<cv::Vec3b>(y, x)[2] = 0;
		// 		}
		// 	}
		// }
        int cx = int(M.m10 / M.m00);
        int cy = int(M.m01 / M.m00);
        cv::circle(image, cv::Point(cx, cy), 20, CV_RGB(0, 0, 255), -1);

        int err = cx - width / 2.0;
		ROS_INFO("err = %d", err);
		sumErr += err * dt;

		float P = Kp * err;
		float I = Ki * sumErr;
		float D = Kd * (err - prevErr) / dt;

		float cmd = P + I + D;

		motors(50 + cmd, 50 - cmd);
		prevErr = err;
    }
	else
		motors(50, 50);

    cv::imshow("img", image);
    cv::waitKey(1);
}

void motors(int16_t left, int16_t right)
{
    car_msgs::MotorsControl msg;
    msg.left = truncate(left);
    msg.right = truncate(right);
    pub.publish(msg);
}

int16_t truncate(int16_t pwm)
{
    if(pwm < -MAX_PWM)
        return -MAX_PWM;
    if(pwm > MAX_PWM)
        return MAX_PWM;
    return pwm;
}
