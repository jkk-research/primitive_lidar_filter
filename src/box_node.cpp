#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <primitive_lidar_filter/primitive_lidar_filter_core.hpp>
#include <primitive_lidar_filter/MyParamsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>

pcl::PointCloud<pcl::PointXYZ> pcl_points;
ros::Publisher pcl_pub, marker_pub;
double position_x, position_y, position_z, length_x, length_y, length_z;
std::string topic_name_in, topic_name_out;

void paramsCallback(my_dyn_rec::MyParamsConfig &config, uint32_t level)
{
  position_x = config.position_x;
  position_y = config.position_y;
  position_z = config.position_z;
  length_x = config.length_x;
  length_y = config.length_y;
  length_z = config.length_z;
  topic_name_in = config.topic_name_in;
  topic_name_out = config.topic_name_out;
  ROS_INFO_STREAM("[" << position_x << ", " << position_y << ", " << position_z << "] {" << length_x << ", " << length_y << ", " << length_z << "} ");
}


void cloud_callback(const pcl::PCLPointCloud2 &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered, pcl_points;
    pcl::fromPCLPointCloud2(cloud_msg, pcl_points);
    cloud_filtered.header = cloud_msg.header;
    for (auto cloud_point : pcl_points)
    {
        if (cloud_point.x > position_x - (length_x / 2) && cloud_point.x < position_x + (length_x / 2)
         && cloud_point.y > position_y - (length_y / 2) && cloud_point.y < position_y + (length_y / 2) 
         && cloud_point.z > position_z - (length_z / 2) && cloud_point.z < position_z + (length_z / 2) 
         ){
            ;
        }
        else{
            cloud_filtered.push_back(cloud_point);
        }
    }
    pcl_pub.publish(cloud_filtered);
    visualization_msgs::Marker marker;
    marker.header.frame_id = cloud_msg.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "debug";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position_x;
    marker.pose.position.y = position_y;
    marker.pose.position.z = position_z;
    marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
    marker.scale.x = length_x; marker.scale.y = length_y; marker.scale.z = length_z;
    marker.color.r = 0.2f;
    marker.color.g = 0.8f;
    marker.color.b = 0.2f;
    marker.color.a = 0.5;

    // TODO if debug
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "box_primitive_lidar_filter");
    ROS_INFO_STREAM("Node started: " << ros::this_node::getName());
    dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig> server;
    dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig>::CallbackType f;
    f = boost::bind(&paramsCallback, _1, _2);
    server.setCallback(f);

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe(topic_name_in, 1, cloud_callback);
    pcl_pub = n.advertise<pcl::PCLPointCloud2>(topic_name_out, 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("debug_marker", 1);
    ros::spin();
    return 0;
}

