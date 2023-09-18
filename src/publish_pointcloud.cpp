/**********************************************/
/*publish_pointcloud.cpp                      */
/**********************************************/
/*description                                 */
/**********************************************/
/*ROS*/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
/*opencv*/
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
/*realsense*/
#include <librealsense2/rs.hpp>
#include <cv-helpers.hpp>
/*point cloud library*/
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

class Publish_PC
{
    private:
        ros::NodeHandle nh;
        /*set publisher*/
        ros::Publisher pub_pc;

        /*realsense*/
        rs2::pipeline pipeline_;
        rs2::config config_;   
        rs2::align align_;
        rs2::points rs_points_;
        rs2::pointcloud rs_pc_;

        /*ros pointcloud*/
        sensor_msgs::PointCloud2 ros_pc_;
        std::string camera_link_ = "camera_color_optical_frame";

        /*function*/
        void set_config();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame color);
        std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords);

    public:
        void execute_program();

        Publish_PC():align_(RS2_STREAM_COLOR)
        {
            pub_pc = nh.advertise<sensor_msgs::PointCloud2>("rs_pc", 10);
            set_config();
        }
        ~Publish_PC()
        {
            cv::destroyAllWindows();
            pipeline_.stop();
        }

};

void Publish_PC::set_config()
{
    config_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    config_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Publish_PC::points_to_pcl(const rs2::points& points, const rs2::video_frame color)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width=sp.width();
    cloud->height=sp.height();
    cloud->is_dense=false;
    cloud->points.resize(points.size());

    auto tex_coords = points.get_texture_coordinates();
    auto ptr = points.get_vertices();

    for(auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;

        std::tuple<uint8_t, uint8_t, uint8_t> current_color;
        current_color = get_texcolor(color, *tex_coords); 
        p.r = std::get<2>(current_color);
        p.g = std::get<1>(current_color);
        p.b = std::get<0>(current_color);

        ptr++;
        tex_coords++;
    }
    
    return cloud;    
}

std::tuple<uint8_t, uint8_t, uint8_t> Publish_PC::get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
    const int w = texture.get_width(), h = texture.get_height();
    
    // convert normals [u v] to basic coords [x y]
    int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);

    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx+1], texture_data[idx+2]);
}

void Publish_PC::execute_program()
{
    pipeline_.start(config_);
    std::cout << "start!!!" << std::endl;

    while (ros::ok())
    {
        rs2::frameset frames = pipeline_.wait_for_frames();
        rs2::frameset aligned_frames = align_.process(frames);

        rs2::video_frame color_frame = aligned_frames.get_color_frame();
        rs2::video_frame depth_frame = aligned_frames.get_depth_frame();

        /*create poitcloud*/
        rs_pc_.map_to(color_frame);
        rs_points_ = rs_pc_.calculate(depth_frame);

        auto pcl_points = points_to_pcl(rs_points_, color_frame);
        pcl::toROSMsg(*pcl_points, ros_pc_);
        ros_pc_.header.frame_id = camera_link_;
        ros_pc_.header.stamp = ros::Time::now();

        pub_pc.publish(ros_pc_);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_pointcloud");
    
    Publish_PC publish_pc;
    publish_pc.execute_program();

    std::cout << "program has finished" << std::endl;
    return 0;
}