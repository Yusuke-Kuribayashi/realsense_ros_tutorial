/**********************************************/
/*publish_image.cpp                           */
/**********************************************/
/*description                                 */
/**********************************************/
/*ROS*/
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
/*opencv*/
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
/*realsense*/
#include <librealsense2/rs.hpp>
#include <cv-helpers.hpp>

class Publish_Image
{
    private:
        ros::NodeHandle nh;
        /*set publisher*/
        ros::Publisher pub_color_image;
        ros::Publisher pub_depth_image;
        ros::Publisher pub_ir_image;

        /*realsense*/
        rs2::pipeline pipeline_;
        rs2::config config_;
        rs2::colorizer color_map_;

        /*opencv image*/
        cv::Mat color_image_;
        cv::Mat depth_image_;
        cv::Mat ir_image_;

        /*ros image*/
        sensor_msgs::ImagePtr ros_color_image_;
        sensor_msgs::ImagePtr ros_depth_image_;
        sensor_msgs::ImagePtr ros_ir_image_;


        /*function*/
        void set_config();

    public:

        void execute_program();

        Publish_Image()
        {
            pub_color_image = nh.advertise<sensor_msgs::Image>("rs_color_image", 10);
            pub_depth_image = nh.advertise<sensor_msgs::Image>("rs_depth_image", 10);
            pub_ir_image    = nh.advertise<sensor_msgs::Image>("rs_ir_image", 10);

            set_config();
        }
        ~Publish_Image()
        {
            pipeline_.stop();
            cv::destroyAllWindows();
        }


};

void Publish_Image::set_config()
{
    config_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    config_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    config_.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
}

void Publish_Image::execute_program()
{
    pipeline_.start(config_);
    std::cout << "start!!!" << std::endl;

    while (ros::ok())
    {
        rs2::frameset frames = pipeline_.wait_for_frames();

        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::video_frame depth_frame = frames.get_depth_frame();
        rs2::video_frame ir_frame    = frames.get_infrared_frame();

        color_image_ = frame_to_mat(color_frame);
        depth_image_ = frame_to_mat(depth_frame.apply_filter(color_map_));
        ir_image_    = frame_to_mat(ir_frame);

        ros_color_image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image_).toImageMsg();
        ros_depth_image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_image_).toImageMsg();
        ros_ir_image_    = cv_bridge::CvImage(std_msgs::Header(), "mono8", ir_image_).toImageMsg();

        cv::imshow("color image", color_image_);
        cv::imshow("depth image", depth_image_);
        cv::imshow("ir image", ir_image_);
        
        if(cv::waitKey(1) == 27){
            break;
        }

        pub_color_image.publish(ros_color_image_);
        pub_depth_image.publish(ros_depth_image_);
        pub_ir_image.publish(ros_ir_image_);
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_image");
    Publish_Image publish_image;

    publish_image.execute_program();

    std::cout << "program finish" << std::endl;
    return 0;
}