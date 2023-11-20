#ifndef DISTURBING_POINTS_DETECTOR
#define DISTURBING_POINTS_DETECTOR

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <optional>

namespace disturbing_points_detector
{
    struct Param
    {
        int hz;

        float max_laser_range;
        // 地図点群とこの距離以内にある点群は地図点群とみなし，filteringする
        // 正確には，この距離の地図におけるピクセル数を計算し，
        // 観測点の周囲のマンハッタン距離でこのピクセル分以内に地図点群における占有点があったとき
        // その観測点は地図のものであるとみなす
        float not_disturbing_torelance_dist; 

        std::string laser_topic_name; 
        std::string map_topic_name;

        std::string laser_frame;
        std::string map_frame;

    };

    class DisturbingPointsDetector
    {
        public:
            DisturbingPointsDetector(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
            void process();
        private:
            void laser_callback(const sensor_msgs::LaserScanConstPtr &msg);
            void map_callback(const nav_msgs::OccupancyGridConstPtr &msg);

            static geometry_msgs::Point create_point_from_laser(float range, float angle);
            static int coordinate_to_map_index(float x, float y, const nav_msgs::OccupancyGrid &map);
            static bool occupancy_check(const nav_msgs::OccupancyGrid &map, int map_index,
                                        float not_disturbing_torelance_dist);
            static bool is_valid_index(int index, const nav_msgs::OccupancyGrid &map);

            void filter_laser(sensor_msgs::LaserScan &scan);

            Param param_;

            std::optional<sensor_msgs::LaserScan> laser_;
            std::optional<nav_msgs::OccupancyGrid> map_;

            ros::Subscriber laser_sub_;
            ros::Subscriber map_sub_;
            ros::Publisher filtered_laser_pub_;

            tf2_ros::Buffer tf_buffer_;
            geometry_msgs::TransformStamped transform_; // tf laser_frame to map_frame
    };
}
#endif
