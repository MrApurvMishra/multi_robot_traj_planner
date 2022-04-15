#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Eigen>
#include <math.h>
#include <random>

//ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

//Octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <mission.hpp>

// using standard namespace
using namespace std;

// declaring a local map using KdTree to locate 3D points
pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;

// to be used to get seed for random number generator
random_device rd;

//default random generator seeded with 'rd'
default_random_engine eng(rd());

// decalaring uniformly distributed random floating-point variables
uniform_real_distribution<double>  rand_x;
uniform_real_distribution<double>  rand_y;
uniform_real_distribution<double>  rand_w;
uniform_real_distribution<double>  rand_h;

// defining ROS publisher
ros::Publisher all_map_pub;

// to contain dynamic size arrays
vector<double> _state;

// number of obstacles
int obs_num;

// declaring default value variables
double margin;
double x_min, y_min, z_min, x_max, y_max, z_max;
double r_min, r_max, h_min, h_max, resolution;

// message to contain collection of points
sensor_msgs::PointCloud2 globalMap_pcd;

// for storing collections of 3D points
pcl::PointCloud<pcl::PointXYZ> cloudMap;

// function to generate map
void RandomMapGenerate(const SwarmPlanning::Mission& mission, double x_inc)
{
    double numel_e = 0.00001;
    cloudMap.points.clear();        // for storing collections of 3D points
    pcl::PointXYZ pt_random;        // 3D point to be stored in 'cloudMap'

    // creasting uniform distributions for map axes and obstacle's radius and height
    rand_x = uniform_real_distribution<double>(x_min, x_max);
    rand_y = uniform_real_distribution<double>(y_min, y_max);
    rand_w = uniform_real_distribution<double>(r_min, r_max);
    rand_h = uniform_real_distribution<double>(h_min, h_max);

    // obstacles inside
    int obs_iter = 0;
    while(obs_iter < obs_num)
    {
        // defining obstacles positions
        double x, y, w, h;
        if (obs_iter%2==0)
            x=-x_inc;               // obstacles on left side
        else
            x=x_inc;                // obstacles on right side
        if (obs_iter%3==0)
            y=0;                    // obstacles on y = 0
        else if(obs_iter%3==1)
            y=2;                    // obstacles on upper part
        else y=-2;                  // obstacles on lower part

        x = floor(x/resolution) * resolution + resolution / 2.0;
        y = floor(y/resolution) * resolution + resolution / 2.0;

        // width and length of obstacles
        int widNum = ceil(0.6/resolution);
        int longNum = ceil(2.0/resolution);

        for(int r = -longNum/2.0; r < longNum/2.0; r++ ) {
            for (int s = -widNum/2.0; s < widNum/2.0; s++) {
                h = rand_h(eng);
                int heiNum = ceil(h / resolution);
                for (int t = 0; t < heiNum; t++) {
                    pt_random.x = x + (r + 0.5) * resolution + numel_e;
                    pt_random.y = y + (s + 0.5) * resolution + numel_e;
                    pt_random.z = (t + 0.5) * resolution + numel_e;
                    cloudMap.points.push_back(pt_random);
                }
            }
        }

        obs_iter++;
    }

    // boundary obstacles
    obs_iter = 0;
    while(obs_iter < obs_num)
    {
        double x, y, w, hh,h;
        if (obs_iter==0)
        {
            x=-6;
            y=0;
            w=10;
            hh=0.05;
        }
        else if (obs_iter==1)
        {
            x=6;
            y=0;
            w=10;
            hh=0.05;
        }
        else if (obs_iter==2)
        {
            x=-3.5;
            y=5;
            w=0.05;
            hh=5;
        }
        else if (obs_iter==3)
        {
            x=3.5;
            y=5;
            w=0.05;
            hh=5;
        }
        else if (obs_iter==4)
        {
            x=-3.5;
            y=-5;
            w=0.05;
            hh=5;
        }
        else if (obs_iter==5)
        {
            x=3.5;
            y=-5;
            w=0.05;
            hh=5;
        }

        x = floor(x/resolution) * resolution + resolution / 2.0;
        y = floor(y/resolution) * resolution + resolution / 2.0;

        int widNum = ceil(w/resolution);
        int longNum = ceil(hh/resolution);

        for(int r = -longNum/2.0; r < longNum/2.0; r ++ ) {
            for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
                h = rand_h(eng);
                int heiNum = ceil(0.2 / resolution);
                for (int t = 0; t < heiNum; t++) {

                    // defifning each point
                    pt_random.x = x + (r + 0.5) * resolution + numel_e;
                    pt_random.y = y + (s + 0.5) * resolution + numel_e;
                    pt_random.z = (t + 0.5) * resolution + numel_e;

                    // pushing to the point cloud
                    cloudMap.points.push_back(pt_random);
                }
            }
        }
        // increment for next obstacle
        obs_iter++;
    }
    // updating dimension of point cloud in number of points
    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    ROS_WARN("Finished generate random map ");

    // pointer to input data to 'kdtreeLocalMap'
    // which will be a shared pointer to deep copy of 'cloudMap'
    kdtreeLocalMap.setInputCloud( cloudMap.makeShared() );
}

// publishing 3D points to topic 'all_map_pub' via 'globalMap_pcd' message
void pubSensedPoints()
{
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
    all_map_pub.publish(globalMap_pcd);
}

// main function
int main (int argc, char** argv) {

    // initialize the node
    ros::init(argc, argv, "random_map_generator");
    ros::NodeHandle n( "~" );
    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    // create a publisher
    all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/random_map_generator/all_map", 1);

    // default parameters
    n.param<double>("world/x_min", x_min, -10);
    n.param<double>("world/y_min", y_min, -10);
    n.param<double>("world/z_min", z_min, 0);
    n.param<double>("world/x_max", x_max, 10);
    n.param<double>("world/y_max", y_max, 10);
    n.param<double>("world/z_max", z_max, 2.5);
    n.param<double>("world/margin", margin, 1.5);

    n.param<int>("world/obs_num", obs_num,  6);
    n.param<double>("world/resolution",  resolution, 0.1);
    n.param<double>("world/r_min", r_min,   0.3);
    n.param<double>("world/r_max", r_max,   0.8);
    n.param<double>("world/h_min", h_min,   1.0);
    n.param<double>("world/h_max", h_max,   2.5);

    // defined in 'mission.hpp' in 'include' folder
    // mostly, checks for the mission file, i.e. a JSON with agents' definition
    SwarmPlanning::Mission mission;
    if(!mission.setMission(n)){
        return -1;
    }

    // generate map msg
    // initializes the map
    double x_shift = 3.5;
    RandomMapGenerate(mission, x_shift);

    // publishing points
    ros::Rate rate(10);         // frequency at which publishing is done
    int count = 0;              // to publish only when needed
    int add_obs = 0;            // to add obstacle once
    x_shift = 0.0;              // new position to add obstacles - can be modified and added as argument
    while (ros::ok())
    {
        // to be ran once only
        // adds new obstacles        
        if (add_obs == 1) {
            // generate map msg
            RandomMapGenerate(mission, x_shift);
            ROS_INFO_STREAM("shifting X to " << x_shift);
            add_obs = 2;
        }

        // publishing the map points
        if(count < 1) {
            pubSensedPoints();
            count++;
        }
        // re-running once to incorporate more obstacles
        else if (count == 500 and add_obs == 0) {
            count = 0;
            add_obs = 1;
        } else {
            count++;
        }
        ros::spinOnce();
        rate.sleep();
    }
}
