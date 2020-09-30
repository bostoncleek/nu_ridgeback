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
  LidarAssembler(const ros::NodeHandle& nh) : nh_(nh), first_scan_(false) /*, first_cloud_(false)*/
  {
    loadParameters();

    laser_client_ = nh_.serviceClient<laser_assembler::AssembleScans2>(laser_srv_name_);
    // cloud_client_ = nh_.serviceClient<laser_assembler::AssembleScans2>(cloud_srv_name_);

    laser_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("assembled_scan_cloud", 1, false);
    // assembled_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("assembled_point_cloud", 1, false);

    laser_timer_ =
        nh_.createTimer(ros::Duration(static_cast<double>(1.0 / laser_rate_)), &LidarAssembler::laserCallback, this);
    // cloud_timer_ = nh_.createTimer(ros::Duration(static_cast<double>(1.0/cloud_rate_)),
    // &LidarAssembler::cloudCallback, this);
  }

private:
  void loadParameters()
  {
    ros::NodeHandle pnh("~");
    size_t errors = 0;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "laser_assembler_srv", laser_srv_name_);
    // errors += !rosparam_shortcuts::get(LOGNAME, pnh, "point_cloud_assembler_srv", cloud_srv_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "laser_rate_", laser_rate_);
    // errors += !rosparam_shortcuts::get(LOGNAME, pnh, "cloud_rate_", cloud_rate_);
    rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

    ros::service::waitForService(laser_srv_name_);
    // ros::service::waitForService(cloud_srv_name_);

    ROS_INFO_NAMED(LOGNAME, "Calling srv %s", laser_srv_name_.c_str());
    // ROS_INFO_NAMED(LOGNAME, "Calling srv %s", cloud_srv_name_.c_str()) ;
  }

  void laserCallback(const ros::TimerEvent& event)
  {
    // Don't convert and merge 2D scans to point cloud on first call
    // Need a start and an end time
    if (first_scan_)
    {
      first_scan_ = false;
      return;
    }

    // Merge 2D scans into a point cloud
    laser_assembler::AssembleScans2 srv;
    srv.request.begin = event.last_real;
    srv.request.end = event.current_real;

    // ROS_INFO_NAMED(LOGNAME, "Start: %f", event.last_real.toSec()) ;
    // ROS_INFO_NAMED(LOGNAME, "End: %f", event.current_real.toSec()) ;

    if (laser_client_.call(srv))
    {
      laser_cloud_pub_.publish(srv.response.cloud);
    }
    else
    {
      ROS_ERROR_NAMED(LOGNAME, "Error making service call laser_assembler\n");
    }
  }

  // void cloudCallback(const ros::TimerEvent& event)
  // {
  //   // Don't merge 3D point clouds on first call
  //   // Need a start and an end time
  //   if (first_cloud_)
  //   {
  //     first_cloud_ = false;
  //     return;
  //   }
  //
  //
  //   if(!ros::service::exists(cloud_srv_name_, true))
  //   {
  //     ROS_ERROR_NAMED(LOGNAME, "cloud Not exists\n") ;
  //     return;
  //   }
  //
  //
  //   // Merge 2D scans into a point cloud
  //   laser_assembler::AssembleScans2 srv;
  //   srv.request.begin = event.last_real;
  //   srv.request.end   = event.current_real;
  //
  //   // ROS_INFO_NAMED(LOGNAME, "Start: %f", event.last_real.toSec()) ;
  //   // ROS_INFO_NAMED(LOGNAME, "End: %f", event.current_real.toSec()) ;
  //
  //   if (cloud_client_.call(srv))
  //   {
  //     assembled_cloud_pub_.publish(srv.response.cloud);
  //   }
  //   else
  //   {
  //     ROS_ERROR_NAMED(LOGNAME, "Error making service call to point_cloud2_assembler\n") ;
  //   }
  // }

private:
  ros::NodeHandle nh_;

  bool first_scan_, first_cloud_;
  double laser_rate_, cloud_rate_;
  std::string laser_srv_name_, cloud_srv_name_;

  ros::Publisher laser_cloud_pub_, assembled_cloud_pub_;
  ros::ServiceClient laser_client_, cloud_client_;

  ros::Timer laser_timer_, cloud_timer_;
};
}  // namespace assemble_scans

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
