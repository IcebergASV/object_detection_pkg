#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <ros/console.h>
#include <geometry_msgs/Vector3.h>
#include <fstream>
#include <cstdlib>
#include <std_msgs/String.h>
#include "DetectedObject.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <iostream>
#include <cstdio>
// #include <nlohmann/json.hpp>

// using json = nlohmann::json; // Define an alias for nlohmann::json

class Image_Finder {
public:
    
    Image_Finder() : nh_("") {
        // Subscribe to camera topic publishing data
        sub_img_ = nh_.subscribe("/camera/color/image_raw", 1, &Image_Finder::imageCallback, this);
        publisher = nh_.advertise<geometry_msgs::Vector3>("/raw_image_copy", 1);
    }

    // Function to continue the life of the node
    void spin() {
        ros::Rate rate(100); // 2Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_img_;
    ros::Publisher publisher;

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        if (msg->data.empty()) {
            ROS_WARN("Received image message with empty data. Not saving the image.");
            return; // Exit early if the image data is empty
        }

        // Convert the sensor_msgs/Image to an OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Save the OpenCV image to a file
        std::string image_filename = "/home/david/Documents/catkin_ws/src/object_detection_pkg/temp_files/image.png";  // Replace with your desired file path
        if (cv::imwrite(image_filename, cv_ptr->image)) {
            ROS_INFO("Saved image to %s", image_filename.c_str());

            // Run the Python script and capture its output
            std::string python_command = "python3 /home/david/Documents/catkin_ws/src/object_detection_pkg/src/yolov5_image_analyzer.py " + image_filename;
            FILE* pipe = popen(python_command.c_str(), "r");
            if (!pipe) {
                ROS_ERROR("Failed to execute the Python script.");
                return;
            }

            char buffer[128];
            std::string script_output = "";
            while (fgets(buffer, sizeof(buffer), pipe) != NULL) {
                script_output += buffer;
            }
            pclose(pipe);

            // TODO Parse the JSON output here (script_output variable contains the JSON data)
            //parseAndProcessJSONOutput(script_output);
        } else {
            ROS_ERROR("Failed to save image to %s", image_filename.c_str());
        }

        // You can also publish image info if needed
        publishImageInfo(msg);
    }

    void publishImageInfo(const sensor_msgs::Image::ConstPtr& msg) {
        // Create a Vector3 message to publish image info
        geometry_msgs::Vector3 image_info_msg;
        image_info_msg.x = static_cast<double>(msg->width);
        image_info_msg.y = static_cast<double>(msg->height);
        // Assuming you want to publish encoding information in the z field
        // Modify this part if you want to publish other information
        image_info_msg.z = 0.0; // You can replace 0.0 with encoding information

        // Publish the message
        publisher.publish(image_info_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detection");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    Image_Finder image_finder;
    image_finder.spin();

    return 0;
}
