#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/filters/voxel_grid.h>

class RacetrackMapper : public rclcpp::Node {
public:
    RacetrackMapper() : Node("racetrack_mapper"), global_cloud_(new pcl::PointCloud<pcl::PointXYZI>()), pose_received_(false) {
        // Pontfelhő adatainak publikálása
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/racetrack/map_data", rclcpp::SystemDefaultsQoS());
        
        // Jármű pozíció adatainak subscription-je
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/a2rl/state_estimation/ego_loc_fused", 10,
            std::bind(&RacetrackMapper::poseCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Pose subscription created.");
    
        // LiDAR pontfelhő adatainak subscription-je
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/lidar_front/points", 10,
            std::bind(&RacetrackMapper::lidarCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "LiDAR subscription created.");
        
        RCLCPP_INFO(this->get_logger(), "RacetrackMapper node has been started.");
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        // Jármű pozíciójának frissítése
        current_pose_ = *msg;
        pose_received_ = true;
        // RCLCPP_INFO(this->get_logger(), "Pose data received: [%f, %f, %f]", 
        //             current_pose_.pose.pose.position.x, 
        //             current_pose_.pose.pose.position.y, 
        //             current_pose_.pose.pose.position.z);
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!pose_received_) { // Ellenőrizzük, hogy a jármű pozíciója elérhető-e
            // RCLCPP_WARN(this->get_logger(), "No pose data available yet.");
            return;  
        }
        RCLCPP_INFO(this->get_logger(), "LiDAR data received.");

        // Konvertáljuk a PointCloud2 üzenetet PCL pontfelhővé
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);
        
        // Új pontfelhő létrehozása a transzformált pontoknak 
        pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
        
        // Jármű pozíciója és orientációja
        double px = current_pose_.pose.pose.position.x;
        double py = current_pose_.pose.pose.position.y;
        double pz = current_pose_.pose.pose.position.z;
        tf2::Quaternion q(
            current_pose_.pose.pose.orientation.x,
            current_pose_.pose.pose.orientation.y,
            current_pose_.pose.pose.orientation.z,
            current_pose_.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // RCLCPP_INFO(this->get_logger(), "Processing LiDAR data with pose: [%f, %f, %f], Yaw: %f", px, py, pz, yaw);
        // RCLCPP_INFO(this->get_logger(), "roll, pitch, yaw: [%f, %f, %f]", roll, pitch, yaw);

        // Pontok átalakítása a megadott koordinátarendszer szerint
        for (const auto& point : pcl_cloud.points) {
            pcl::PointXYZI transformed_point;
            transformed_point.x = point.z - 0.63; // LiDAR z -> jármű x (0.63 méterrel hátrébb)
            transformed_point.y = -point.y;       // LiDAR y -> jármű bal (y negatív, hogy jobbra mutasson)
            transformed_point.z = point.x;        // LiDAR x -> jármű felfele (z)
            transformed_point.intensity = point.intensity;
            
            // Pontok átalakítása a jármű globális pozíciójára és orientációjára
            double gx = px + (transformed_point.x * cos(yaw) - transformed_point.y * sin(yaw));
            double gy = py + (transformed_point.x * sin(yaw) + transformed_point.y * cos(yaw));
            double gz = pz + transformed_point.z;

            // RCLCPP_INFO(this->get_logger(), "Check trigo: [%f, %f, %f]", gx, gy, gz);

            // Globális pont
            pcl::PointXYZI global_point;
            global_point.x = gx;
            global_point.y = gy;
            global_point.z = gz;
            global_point.intensity = transformed_point.intensity;

            // RCLCPP_INFO(this->get_logger(), "Transformed points: [%f, %f, %f]", global_point.x, global_point.y, global_point.z);

            global_cloud_->points.push_back(global_point);
        }
        // RCLCPP_INFO(this->get_logger(), "Transformed points and added to global cloud.");

        // Szűrés a számításigény optimalizálása érdekében
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(global_cloud_);
        voxel_filter.setLeafSize(2.0f, 2.0f, 2.0f); // 2 m-es voxel rács
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        voxel_filter.filter(*filtered_cloud);
        global_cloud_ = filtered_cloud;

        // Visszakonvertáljuk a PCL pontfelhőt PointCloud2 üzenetté
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*global_cloud_, output);
        // RCLCPP_INFO(this->get_logger(), "Converted PCL PointCloud to PointCloud2.");
        if (output.data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Converted PointCloud2 message is empty.");
        }       
        else {
            RCLCPP_INFO(this->get_logger(), "Converted PointCloud2 message has %zu points.", global_cloud_->points.size());
        }  

        
        // Állítsuk be a header-t az aktuális időbélyegre
        output.header.stamp = this->now();
        output.header.frame_id = "map";

        
        // Publikáljuk az átalakított pontfelhő message-t
        map_pub_->publish(output);
        RCLCPP_INFO(this->get_logger(), "Published transformed LiDAR data.");

        // Felhő törlés számításigény minimalizálása érdekében
        // clearGlobalCloud();
    }
    void clearGlobalCloud() {
        if (global_cloud_->points.size() > MAX_POINTS) {
            global_cloud_->clear();
            RCLCPP_INFO(this->get_logger(), "Global cloud cleared to prevent memory overflow.");
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_cloud_;
    bool pose_received_;
    const size_t MAX_POINTS = 100000; // Max pont szám ami a felhőben tárolható
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RacetrackMapper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


