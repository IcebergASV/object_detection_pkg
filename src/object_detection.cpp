#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <object_detection_pkg/ObjDetected.h>
#include <object_detection_pkg/DetectedObjsArray.h>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <sys/stat.h>
#include <sys/types.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <jsoncpp/json/json.h>
#include "DetectedObject.h"

// Image_Finder class definition
class Image_Finder {
public:
    // Constructor
    Image_Finder() : nh_("") {
        // Subscribing to a camera topic to receive image data
        sub_img_ = nh_.subscribe("/camera/color/image_raw", 1, &Image_Finder::imageCallback, this);
        // Setting up a publisher for the detected objects
        publisher = nh_.advertise<object_detection_pkg::DetectedObjsArray>("/Detected_Objects", 1);
    }

    // Function to continue the life of the node
    void spin() {
        ros::Rate rate(100); // Setting a rate of 100Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_img_;
    ros::Publisher publisher;
    object_detection_pkg::DetectedObjsArray detected_objs_array;

    // Callback function for image data from the subscribed topic
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        if (msg->data.empty()) {
            ROS_WARN("Received image message with empty data. Not saving the image.");
            return; // Exit early if the image data is empty
        }

        // Ensuring the directory for saving images exists
        std::string directory = "/home/david/Documents/catkin_ws/src/object_detection_pkg/temp_files";
        mkdir(directory.c_str(), 0777);
        
        // Converting ROS image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Saving the OpenCV image to a file
        std::string image_filename = directory + "/image.png";
        if (!cv::imwrite(image_filename, cv_ptr->image)) {
            ROS_ERROR("Failed to save image to %s", image_filename.c_str());
            return;
        }
        ROS_INFO("Saved image to %s", image_filename.c_str());

        // Running a Python script for object detection
        std::string python_command = "python3 /home/david/Documents/catkin_ws/src/object_detection_pkg/src/yolov5_image_analyzer.py " + image_filename;
        int return_code = system(python_command.c_str());
        if (return_code != 0) {
            ROS_ERROR("Failed to execute the Python script.");
            return;
        }

        // Waiting for the script to complete and generate JSON output
        ros::Duration(1.0).sleep(); // Adjust the duration if needed

        // Reading the JSON output from the Python script
        std::string json_file_path = "/home/david/Documents/catkin_ws/src/object_detection_pkg/temp_files/detected_objects.json";

        std::ifstream json_file(json_file_path);
        if (!json_file.is_open()) {
            ROS_ERROR("Failed to open JSON file: %s", json_file_path.c_str());
            return;
        }
        std::string json_content((std::istreambuf_iterator<char>(json_file)),
                                std::istreambuf_iterator<char>());

        ROS_INFO("JSON file content: %s", json_content.c_str());
        parseAndProcessJSONOutput(json_content);
    }

    // Function to parse JSON output and process detected objects
    void parseAndProcessJSONOutput(const std::string& json_output) {
        if (json_output.empty() || std::all_of(json_output.begin(), json_output.end(), isspace)) {
            ROS_WARN("JSON output is empty or only contains whitespaces.");
            return;
        }

        ROS_DEBUG("Attempting to parse JSON. Length of string: %lu", json_output.length());
        Json::Value root;
        Json::Reader reader;
        bool parsingSuccessful = reader.parse(json_output, root);
        if (!parsingSuccessful || !root.isArray()) {
            ROS_ERROR("Failed to parse JSON: %s", reader.getFormattedErrorMessages().c_str());
            return;
        }

        if (root.empty()) {
            ROS_INFO("JSON array is empty. No objects detected.");
            return;
        }
        // Parsing each object in the JSON array
        for (const auto& obj : root) {
            object_detection_pkg::ObjDetected detected_obj;
            detected_obj.class_name = obj["class_name"].asString();
            detected_obj.confidence = obj["confidence"].asDouble();
            detected_obj.x_min = obj["x_min"].asInt();
            detected_obj.x_max = obj["x_max"].asInt();
            detected_obj.y_min = obj["y_min"].asInt();
            detected_obj.y_max = obj["y_max"].asInt();
            
            detected_objs_array.objects.push_back(detected_obj);
        }
        // Publishing the array of detected objects and clearing the array for the next callback
        publisher.publish(detected_objs_array);
        detected_objs_array.objects.clear();
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
