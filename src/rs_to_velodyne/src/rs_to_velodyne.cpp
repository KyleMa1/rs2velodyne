
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>        
#include <pcl/common/common.h>  

std::string output_type;

static int RING_ID_MAP_RUBY[] = {
        3, 66, 33, 96, 11, 74, 41, 104, 19, 82, 49, 112, 27, 90, 57, 120,
        35, 98, 1, 64, 43, 106, 9, 72, 51, 114, 17, 80, 59, 122, 25, 88,
        67, 34, 97, 0, 75, 42, 105, 8, 83, 50, 113, 16, 91, 58, 121, 24,
        99, 2, 65, 32, 107, 10, 73, 40, 115, 18, 81, 48, 123, 26, 89, 56,
        7, 70, 37, 100, 15, 78, 45, 108, 23, 86, 53, 116, 31, 94, 61, 124,
        39, 102, 5, 68, 47, 110, 13, 76, 55, 118, 21, 84, 63, 126, 29, 92,
        71, 38, 101, 4, 79, 46, 109, 12, 87, 54, 117, 20, 95, 62, 125, 28,
        103, 6, 69, 36, 111, 14, 77, 44, 119, 22, 85, 52, 127, 30, 93, 60
};
static int RING_ID_MAP_16[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8
};


struct RsPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring = 0;
    double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)(double, timestamp, timestamp))


struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)(float, time, time)
)

struct VelodynePointXYZIR {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIR,
                                   (float, x, x)(float, y, y)
                                           (float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)
)

class RsConverter : public rclcpp::Node {
public:
    RsConverter() : Node("rs_converter") {
        // 添加时间同步参数
        this->declare_parameter("time_offset", 0.0);
        time_offset_ = this->get_parameter("time_offset").as_double();
        

        if (output_type == "XYZI") {
            subRobosensePC = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/rslidar_points", 10, 
                std::bind(&RsConverter::rsHandler_XYZI, this, std::placeholders::_1));
        } else if (output_type == "XYZIRT") {
            subRobosensePC = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/rslidar_points", 10, 
                std::bind(&RsConverter::rsHandler_XYZIRT, this, std::placeholders::_1));
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                "Unsupported input pointcloud type. Currently only support XYZI and XYZIRT.");
            rclcpp::shutdown();
        }
        pubRobosensePC = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points", 10);
        RCLCPP_INFO(this->get_logger(), "Listening to /rslidar_points ......");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subRobosensePC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRobosensePC;
    double time_offset_;  

    void rsHandler_XYZI(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<VelodynePointXYZIR>::Ptr pc_new(new pcl::PointCloud<VelodynePointXYZIR>());
        pcl::fromROSMsg(*pc_msg, *pc);

        pc_new->points.reserve(pc->points.size());  

        for (int point_id = 0; point_id < pc->points.size(); ++point_id) {
            if (has_nan(pc->points[point_id]))
                continue;

            VelodynePointXYZIR new_point;
            new_point.x = pc->points[point_id].x;
            new_point.y = pc->points[point_id].y;
            new_point.z = pc->points[point_id].z;
            new_point.intensity = pc->points[point_id].intensity;
            // remap ring id
            if (pc->height == 16) {
                new_point.ring = RING_ID_MAP_16[point_id / pc->width];
            } else if (pc->height == 128) {
                new_point.ring = RING_ID_MAP_RUBY[point_id % pc->height];
            }
            pc_new->points.push_back(new_point);
        }

        publish_points(pc_new, *pc_msg);
    }

    void rsHandler_XYZIRT(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
        pcl::PointCloud<RsPointXYZIRT>::Ptr pc_in(new pcl::PointCloud<RsPointXYZIRT>());
        pcl::fromROSMsg(*pc_msg, *pc_in);

        if (output_type == "XYZIRT") {
            pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_out(new pcl::PointCloud<VelodynePointXYZIRT>());
            handle_pc_msg<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
            add_ring<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
            add_time<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
            publish_points(pc_out, *pc_msg);
        } else if (output_type == "XYZIR") {
            pcl::PointCloud<VelodynePointXYZIR>::Ptr pc_out(new pcl::PointCloud<VelodynePointXYZIR>());
            handle_pc_msg<RsPointXYZIRT, VelodynePointXYZIR>(pc_in, pc_out);
            add_ring<RsPointXYZIRT, VelodynePointXYZIR>(pc_in, pc_out);
            publish_points(pc_out, *pc_msg);
        } else if (output_type == "XYZI") {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZI>());
            handle_pc_msg<RsPointXYZIRT, pcl::PointXYZI>(pc_in, pc_out);
            publish_points(pc_out, *pc_msg);
        }
    }

    template<typename T>
    bool has_nan(T point) {
        if (std::isnan(static_cast<float>(point.x)) || 
            std::isnan(static_cast<float>(point.y)) || 
            std::isnan(static_cast<float>(point.z))) {
            return true;
        }
        return false;
    }

    template<typename T>
    void publish_points(T &new_pc, const sensor_msgs::msg::PointCloud2 &old_msg) {
        
        new_pc->is_dense = false;
        
        
        rclcpp::Time old_time(old_msg.header.stamp);
        rclcpp::Time adjusted_time = old_time + rclcpp::Duration::from_seconds(time_offset_);

        
        sensor_msgs::msg::PointCloud2 pc_new_msg;
        pcl::toROSMsg(*new_pc, pc_new_msg);
        pc_new_msg.header = old_msg.header;
        pc_new_msg.header.stamp = adjusted_time;
        pc_new_msg.header.frame_id = "velodyne";
        pubRobosensePC->publish(pc_new_msg);
    }

    template<typename T_in_p, typename T_out_p>
    void handle_pc_msg(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                       const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {

        
        for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
            if (has_nan(pc_in->points[point_id]))
                continue;
            T_out_p new_point;
  
            new_point.x = pc_in->points[point_id].x;
            new_point.y = pc_in->points[point_id].y;
            new_point.z = pc_in->points[point_id].z;
            new_point.intensity = pc_in->points[point_id].intensity;

            pc_out->points.push_back(new_point);
        }
    }

    template<typename T_in_p, typename T_out_p>
    void add_ring(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                  const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {

        int valid_point_id = 0;
        for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
            if (has_nan(pc_in->points[point_id]))
                continue;

            pc_out->points[valid_point_id++].ring = pc_in->points[point_id].ring;
        }
    }

    template<typename T_in_p, typename T_out_p>
    void add_time(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                  const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
        int valid_point_id = 0;
        for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
            if (has_nan(pc_in->points[point_id]))
                continue;
            // 修改这里的时间戳计算方式
            double scan_time = pc_in->points[point_id].timestamp - pc_in->points[0].timestamp;
            pc_out->points[valid_point_id++].time = static_cast<float>(scan_time);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    if (argc < 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
            "Please specify input pointcloud type( XYZI or XYZIRT) and output pointcloud type(XYZI, XYZIR, XYZIRT)!!!");
        return 1;
    }
    
    output_type = argv[2];
    auto node = std::make_shared<RsConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}