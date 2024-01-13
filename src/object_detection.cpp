#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <object_detection_pkg/ObjDetected.h>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <jsoncpp/json/json.h>
#include "DetectedObject.h"


class Image_Finder {
public:
    
    Image_Finder() : nh_("") {
        // Subscribe to camera topic publishing data
        sub_img_ = nh_.subscribe("/camera/color/image_raw", 1, &Image_Finder::imageCallback, this);
        publisher = nh_.advertise<object_detection_pkg::ObjDetected>("/Detected_Objects", 1);
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
        std::string image_filename = "/home/david/Documents/catkin_ws/src/object_detection_pkg/temp_files/image.png";
        if (!cv::imwrite(image_filename, cv_ptr->image)) {
            ROS_ERROR("Failed to save image to %s", image_filename.c_str());
            return;
        }

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

        // Read and parse the JSON file
        std::ifstream json_file(script_output);
        std::string json_content((std::istreambuf_iterator<char>(json_file)),
                                std::istreambuf_iterator<char>());
        parseAndProcessJSONOutput(json_content);

        // Publish image info if needed
        publishImageInfo(msg);
    }

    void parseAndProcessJSONOutput(const std::string& json_output) {
        Json::Value root;
        Json::Reader reader;
        bool parsingSuccessful = reader.parse(json_output, root);
        if (!parsingSuccessful) {
            ROS_ERROR("Failed to parse JSON: %s", reader.getFormattedErrorMessages().c_str());
            return;
        }

        for (const auto& obj : root) {
            // Extract information from each JSON object
            std::string class_name = obj["class_name"].asString();
            double confidence = obj["confidence"].asDouble();
            int x_min = obj["x_min"].asInt();
            int x_max = obj["x_max"].asInt();
            int y_min = obj["y_min"].asInt();
            int y_max = obj["y_max"].asInt();

            ROS_INFO("Detected: %s with confidence %f at [%d, %d, %d, %d]",
                    class_name.c_str(), confidence, x_min, x_max, y_min, y_max);

            // Here you can also populate and publish a custom ROS message if required
        }
    }

    void publishImageInfo(const sensor_msgs::Image::ConstPtr& msg) {
        // Create an instance of the custom message
        object_detection_pkg::ObjDetected custom_message;

        // Fill in the custom message fields with hard-coded values
        custom_message.class_name = "Buoy";
        custom_message.confidence = 0.48662763833999634;
        custom_message.x_min = 0;
        custom_message.x_max = 113;
        custom_message.y_min = 235;
        custom_message.y_max = 450;

        // Publish the custom message
        publisher.publish(custom_message);
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
