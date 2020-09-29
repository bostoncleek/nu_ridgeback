/**
* @file assemble_scans_node.cpp
* @author Boston CLeek
* @date 29 Sep 2020
* @brief Combines 2D and 3D lidar scans into a single point cloud
*/

#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <laser_assembler/AssembleScans2.h>


constexpr char LOGNAME[] = "assemble_scans";


int main(int argc, char** argv)
{
  ros::init(argc, argv, "assemble_scans");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string laser_srv_name = "";
  std::string cloud_srv_name = "";
  double rate = 0.0;

  pnh.getParam("laser_assembler_srv", laser_srv_name);
  pnh.getParam("point_cloud_assembler_srv", cloud_srv_name);
  pnh.getParam("rate", rate);

  ros::ServiceClient scan_client = nh.serviceClient<laser_assembler::AssembleScans2>(laser_srv_name);
  ros::ServiceClient cloud_client = nh.serviceClient<laser_assembler::AssembleScans2>(laser_srv_name);

  laser_assembler::AssembleScans2 scans_srv;
  laser_assembler::AssembleScans cloud_srv;

  ros::Rate loop_rate(rate);
  while(nh.ok())
  {



    loop_rate.sleep();
  }

  return 0;
}








//
