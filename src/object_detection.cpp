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
            detectAndPublishObjects()
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
        //Save image message
        img_msg_ = *msg;

        // // Use ROS_DEBUG_STREAM for debugging
        // ROS_DEBUG_STREAM("Received an image with width=" << image_width << " and height=" << image_height);

        geometry_msgs::Vector3 prop_coords_msg;

    }

    void detectAndPublishObjects(){
        std::string temp_img_path = "temp_image.jpg";
        saveImageToFile(temp_img_path);

        std::string command "python3 TODO add path to objectdetcitonyolov5.py";
        std::string result = executeCommand(command);

        //Process/Pars the result from the python file depending on how the information is received

        //Map it to the custom message
    }

    void saveImageToFile(const std::string& image_path){
        std::ofstream image_file(image_path, std::ios::out | std::ios::binary);
        image_file.write(reinterpret_cast<const char*>(image_data_matrix.data()), image_data_matrix_.size());
        image_file.close();
    }

    std::string executeCommand(const std::string& command){
        FILE* pipe = popen(command.c_star(),"r");
        if (!pipe){
            return "Error";
        }
        char buffer[128];
        std::string result = "";
        while(!feof(pipe)){
            if (fgeets(buffer,128,pipe)!= NULL){
                result += buffer;
            }
        }
        pclose(pipe);
        return result:
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