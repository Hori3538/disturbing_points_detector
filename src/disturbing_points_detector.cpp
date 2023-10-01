#include <disturbing_points_detector/disturbing_points_detector.hpp>

namespace disturbing_points_detector
{
    DisturbingPointsDetector::DisturbingPointsDetector(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    {
        private_nh.param<int>("hz", param_.hz, 30);
        private_nh.param<float>("max_laser_range", param_.max_laser_range, 2.0);
        private_nh.param<int>("offset_pixel", param_.offset_pixel, 3);
        private_nh.param<std::string>("laser_topic_name", param_.laser_topic_name, "/scan");
        private_nh.param<std::string>("map_topic_name", param_.map_topic_name, "/map");
        private_nh.param<std::string>("laser_frame", param_.laser_frame, "laser");
        private_nh.param<std::string>("map_frame", param_.map_frame, "map");

        laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>(param_.laser_topic_name, 5, &DisturbingPointsDetector::laser_callback, this);
        map_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>(param_.map_topic_name, 1, &DisturbingPointsDetector::map_callback, this);
        filtered_laser_pub_ = nh.advertise<sensor_msgs::LaserScan>("/disturbing_points_detector/filtered_laser", 1);
        point_pub_ = nh.advertise<geometry_msgs::PointStamped>("/disturbing_points_detector/point", 1);
    }
    void DisturbingPointsDetector::laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
    {
        laser_ = *msg;
    }

    void DisturbingPointsDetector::map_callback(const nav_msgs::OccupancyGridConstPtr &msg)
    {
        map_ = *msg;
    }

    geometry_msgs::Point DisturbingPointsDetector::create_point_from_laser(float range, float angle)
    {
        geometry_msgs::Point point;
        point.x = range * cos(angle);
        point.y = range * sin(angle);

        return point;
    }

    int DisturbingPointsDetector::coordinate_to_map_index(float x, float y, nav_msgs::OccupancyGrid &map)
    {
        int index_x = int((x - map.info.origin.position.x) / map.info.resolution);
        int index_y = int((y - map.info.origin.position.y) / map.info.resolution);

        return index_x + index_y * map.info.width;
    }


    bool DisturbingPointsDetector::is_within_range(int index, nav_msgs::OccupancyGrid &map)
    {
       if(index < 0) return false;
       if(index >= map.info.height * map.info.width) return false;
       return true;
    }

    bool DisturbingPointsDetector::occupancy_check(nav_msgs::OccupancyGrid &map, int map_index,
            int offset_pixel)
    {
        for(int dx_pixel = -offset_pixel; dx_pixel <= offset_pixel; dx_pixel++)
        {
            for(int dy_pixel = -offset_pixel; dy_pixel <= offset_pixel; dy_pixel++)
            {
                const int target_pixel = map_index + dx_pixel + dy_pixel*map.info.width;
                if(!is_within_range(target_pixel, map)) continue;

                if(map.data[target_pixel] == 100) return true;
            }
        }
        return false;
    }

    // To Do この関数でかすぎるので分解する
    void DisturbingPointsDetector::filter_laser(sensor_msgs::LaserScan &scan)
    {
        for(int i=0, size=scan.ranges.size(); i<size; i++)
        {
            float &range = scan.ranges[i];
            if(range > param_.max_laser_range)
            {
                range = NAN;
                continue;
            }
            const float angle = i * scan.angle_increment + scan.angle_min;

            geometry_msgs::PointStamped point_stamped;
            point_stamped.header = scan.header;
            point_stamped.point = create_point_from_laser(range, angle);
            tf2::doTransform(point_stamped, point_stamped, transform_);
            point_stamped.header.frame_id = param_.map_frame;

            point_pub_.publish(point_stamped);

            int map_index_of_point = coordinate_to_map_index(point_stamped.point.x, point_stamped.point.y, map_.value());
            bool occupancy_flag = occupancy_check(map_.value(), map_index_of_point, param_.offset_pixel); 

            if(occupancy_flag)
            {
                range = NAN;
                continue;
            } 
        }
    }

    void DisturbingPointsDetector::process()
    {
        ros::Rate loop_rate(param_.hz);
        tf2_ros::TransformListener tf_listener(tf_buffer_);

        while (ros::ok())
        {
            if(map_.has_value() && laser_.has_value())
            {
                try
                {
                    transform_ = tf_buffer_.lookupTransform(param_.map_frame, param_.laser_frame, ros::Time(0));
                }
                catch(tf2::TransformException &ex)
                {
                    ROS_WARN("%s", ex.what());
                    continue;
                }
                filter_laser(laser_.value());
                filtered_laser_pub_.publish(laser_.value());

                laser_.reset();
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}