#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <ros/console.h>
#include <geometry_msgs/Vector3.h>


class Image_Finder{
public:
    
    Image_Finder(): nh_("") {
        //Subsribe to camera topic publishing data
        sub_img_ = nh_.subscribe("/camera/color/image_raw", 1, &Image_Finder::imageCallback, this);
        publisher = nh_.advertise<geometry_msgs::Vector3>("/raw_image_copy", 1);;
    }

    //Function to continue the life of the node
    void spin(){
        ros::Rate rate(2); // 2Hz
        while (ros::ok()) {
            ros::spinOnce();
            publishImageInfo(); // Publish image information
            rate.sleep();
        }
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_img_;
    ros::Publisher publisher;

    sensor_msgs::Image img_msg_;

    std::vector<uint8_t> image_data_matrix;
    uint32_t image_height;
    uint32_t image_width;
    std::string encoding;

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
        //Access the image data from the message
        image_data_matrix = msg->data;

        //Acces image dimensions
        image_height = msg->height;
        image_width = msg->width;

        //Access image encoding
        encoding = msg->encoding;

        //Save image message
        img_msg_ = *msg;

        // // Use ROS_DEBUG_STREAM for debugging
        // ROS_DEBUG_STREAM("Received an image with width=" << image_width << " and height=" << image_height);

        geometry_msgs::Vector3 prop_coords_msg;

    }

    void publishImageInfo() {
        // Create a Vector3 message to publish image info
        geometry_msgs::Vector3 msg;
        msg.x = static_cast<double>(image_width);
        msg.y = static_cast<double>(image_height);
        // Assuming you want to publish encoding information in the z field
        // Modify this part if you want to publish the image data
        msg.z = 0.0; // You can replace 0.0 with encoding information

        // Publish the message
        publisher.publish(msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detection");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();

    Image_Finder image_finder;
    image_finder.spin();

    return 0;
}