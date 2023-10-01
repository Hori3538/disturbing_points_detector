#include <disturbing_points_detector/disturbing_points_detector.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "disturbing_points_detector_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    disturbing_points_detector::DisturbingPointsDetector disturbing_points_detector(nh, private_nh);

    disturbing_points_detector.process();
    return 0;
}
