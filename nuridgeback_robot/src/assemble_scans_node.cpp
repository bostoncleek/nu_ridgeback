/**
* @file assemble_scans_node.cpp
* @author Boston CLeek
* @date 29 Sep 2020
* @brief Combines 2D and 3D lidar scans into a single point cloud
*/

#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <laser_assembler/AssembleScans.h>
#include <laser_assembler/AssembleScans2.h>



namespace assemble_scans
{
constexpr char LOGNAME[] = "assemble_scans";

class LidarAssembler
{
public:
  LidarAssembler(ros::NodeHandle &nh) : nh_(nh), first_scan_(false)
  {
    loadParameters();

    laser_client_ = nh_.serviceClient<laser_assembler::AssembleScans2>(laser_srv_name_);
    cloud_client_ = nh_.serviceClient<laser_assembler::AssembleScans>(cloud_srv_name_);

    laser_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("assembled_scan_cloud", 1, true);
    assembled_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("assembled_point_cloud", 1, true);

    laser_timer_ = nh_.createTimer(ros::Duration(static_cast<double>(1.0/laser_rate_)), &LidarAssembler::laserCallback, this);
  }

  void loadParameters()
  {
    ros::NodeHandle pnh("~");
    size_t errors = 0;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "laser_assembler_srv", laser_srv_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "point_cloud_assembler_srv", cloud_srv_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "laser_rate_", laser_rate_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "cloud_rate_", cloud_rate_);
    rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
  }

  void laserCallback(const ros::TimerEvent& event)
  {
    // Don't convert 2D scans to point cloud on first call
    // Need a start and an end time
    if (first_scan_)
    {
      first_scan_ = false;
      return;
    }

    // Merge 2D scans into a point cloud
    laser_srv_.request.begin = event.last_real;
    laser_srv_.request.end   = event.current_real;

    if (laser_client_.call(laser_srv_))
    {
      laser_cloud_pub_.publish(laser_srv_.response.cloud);
    }
    else
    {
      ROS_ERROR_NAMED(LOGNAME, "Error making service call to convert laser assembler\n") ;
    }
  }

private:
  bool first_scan_;

  double laser_rate_;
  double cloud_rate_;

  std::string laser_srv_name_;
  std::string cloud_srv_name_;

  ros::NodeHandle nh_;

  ros::Publisher laser_cloud_pub_;
  ros::Publisher assembled_cloud_pub_;

  ros::ServiceClient laser_client_;
  ros::ServiceClient cloud_client_;

  ros::Timer laser_timer_;
  ros::Timer cloud_timer_;

  laser_assembler::AssembleScans2 laser_srv_;
  laser_assembler::AssembleScans cloud_srv_;
};
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "assemble_scans");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  assemble_scans::LidarAssembler assembler(nh);
  ros::waitForShutdown();

  return 0;
}


















//
