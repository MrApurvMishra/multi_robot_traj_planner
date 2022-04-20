// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

// Octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

// Parameters
#include <param.hpp>
#include <mission.hpp>
#include <timer.hpp>

// Submodules
#include <ecbs_planner.hpp>
#include <corridor.hpp>
#include <prioritized_traj_optimization.hpp>
#include <prioritized_traj_publisher.hpp>

bool has_octomap = false;       // check for occupancy map
bool has_path = false;          // check for defined path
bool flag = true;               // check for added obstacles

/*  A shared pointer can be used to point to member objects.
    Octomap used to implement a 3D occupancy grid,
    based on OcTree which is a tree data structure
    which allows each node to have atmost 8 children.   */
//  shared pointer to an octomap object, OcTree is a constructor
std::shared_ptr<octomap::OcTree> octree_obj;

std::vector<double> current_x;
std::vector<double> current_y;

void obsCallback(const std_msgs::Bool& flag_msg) {
    flag = flag_msg.data;
}

// void trajCallback(const nav_msgs::Path& traj_msg, const std::string &topic) {
void trajCallback(const ros::MessageEvent<nav_msgs::Path const>& event) {

    // extracting topic name
    const ros::M_string& header = event.getConnectionHeader();
    std::string topic = header.at("topic");

    // extracting pose data
    const nav_msgs::Path& traj_msg = *event.getMessage();
    geometry_msgs::PoseStamped pose_data = traj_msg.poses.back();

    // string manipulation for agent/robot number
    int len = topic.size();
    char last = topic.back();
    char sec_last = topic[len - 2];
    int n;
    if (isdigit(sec_last)) n = ((int)sec_last - 48)*10 + (int)last - 48;
    else n = (int)last - 48;

    // std::cout << "Pose : " << n << endl << pose_data << endl;
    
    // updating current positions
    current_x[n] = pose_data.pose.position.x;
    current_y[n] = pose_data.pose.position.y;
}

/*  callback function when we subscribe to the '/octomap_full' topic
    which contains all the points making up the 3D occupancy grid   */
void octomapCallback(const octomap_msgs::Octomap& octomap_msg) {
    
    // check if Octomap exists - return if it does
    if(has_octomap)
        return;

    /*  create Octomap, converting data from binary message to occupancy map,
        which is casted as or converted to the OcTree data structure */
    octree_obj.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(octomap_msg)));

    // set to true as we have a map now
    has_octomap = true;
    flag = true;
    // std::cout << flag << endl;
}

// main function
int main(int argc, char* argv[]) {

    // initialize node for trajectory planner
    ros::init (argc, argv, "swarm_traj_planner_rbp");
    ros::NodeHandle nh( "~" );

    // define subscriber for added obstacle flag and the Octomap
    ros::Subscriber obs_added_sub = nh.subscribe( "/path_exists", 1, obsCallback );
    ros::Subscriber octomap_sub = nh.subscribe( "/octomap_full", 1, octomapCallback );

    // defined in 'mission.hpp' in 'include' folder
    // mostly, checks for the mission file, i.e. a JSON with agents' definition
    SwarmPlanning::Mission mission;
    if(!mission.setMission_init(nh)){
        return -1;
    }

    // the number of robots
    int qn = mission.qn;
    
    // define subscriber for the desired trajectory pose data
    std::vector<ros::Subscriber> traj_sub;
    traj_sub.resize(qn);
    current_x.resize(qn);
    current_y.resize(qn);
    for(int n_agent=0; n_agent<qn; n_agent++) {
        std::string mav_name = "/mav" + std::to_string(n_agent);
        ros::Subscriber traj_subscriber = nh.subscribe("/desired_trajectory" + mav_name, 1, trajCallback);
        traj_sub[n_agent] = traj_subscriber;
    }

    // vector for waypoints' arrays for publisher
    way_point_pub.resize(qn);

    // create waypoints' publisher for each robot
    for(int n_agent= 0; n_agent<qn; n_agent++) {
        std::string robot_name = "/robot" + std::to_string(qi+1);
        way_point_pub[qi] = nh.advertise<nav_msgs::Path>(robot_name+"/mpc_predict_all", 1);
    }

    // ROS Parameters
    SwarmPlanning::Param param;
    if(!param.setROSParam(nh)){
        return -1;
    }
    param.setColor(mission.qn);

    // Submodules
    std::shared_ptr<DynamicEDTOctomap> distmap_obj;         // Euclidean distance object
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;   // discrete initial trajectory
    std::shared_ptr<Corridor> corridor_obj;                 // safe-corridor construction
    std::shared_ptr<MPCPlanner> MPCPlanner_obj;             // trajectory optimization
    std::shared_ptr<ResultPublisher> resultPublisher_obj;

    // Main Loop
    ros::Rate rate(20);
    Timer timer_total;
    Timer timer_step;
    double start_time, current_time;

    while (ros::ok()) {
        if(flag) {
            if (has_octomap && !has_path) {

                // Build 3D Euclidean Distance Field
                {
                    float maxDist = 1;
                    octomap::point3d min_point3d(param.world_x_min, param.world_y_min, param.world_z_min);
                    octomap::point3d max_point3d(param.world_x_max, param.world_y_max, param.world_z_max);
                    distmap_obj.reset(new DynamicEDTOctomap(maxDist, octree_obj.get(), min_point3d, max_point3d, false));
                    distmap_obj.get()->update();
                }

                timer_total.reset();
                ROS_INFO("Multi-robot Trajectory Planning");

                // Step 1: Plan Initial Trajectory
                timer_step.reset();
                {
                    initTrajPlanner_obj.reset(new ECBSPlanner(distmap_obj, mission, param));
                    if (!initTrajPlanner_obj.get()->update(param.log)) {
                        return -1;
                    }
                }
                timer_step.stop();
                ROS_INFO_STREAM("ECBS Planner runtime: " << timer_step.elapsedSeconds());

                
                // Step 2: Generate Safe Corridor
                timer_step.reset();
                {
                    corridor_obj.reset(new Corridor(initTrajPlanner_obj, distmap_obj, mission, param));
                    if (!corridor_obj.get()->update(param.log)) {
                        return -1;
                    }
                }
                timer_step.stop();
                ROS_INFO_STREAM("Safe Corrifor runtime: " << timer_step.elapsedSeconds());
                
                
                // Step 3: Formulate NLP problem and solving it to generate trajectory for the robot team
                timer_step.reset();
                {
                    MPCPlanner_obj.reset(new MPCPlanner(corridor_obj, initTrajPlanner_obj, mission, param));
                    if (!MPCPlanner_obj.get()->update(param.log)) {
                        return -1;
                    }
                }
                timer_step.stop();
                ROS_INFO_STREAM("Trajectory Optimization runtime: " << timer_step.elapsedSeconds());
                
                timer_total.stop();
                ROS_INFO_STREAM("Overall runtime: " << timer_total.elapsedSeconds());

                // Plot Planning Result
                resultPublisher_obj.reset(new ResultPublisher(nh, MPCPlanner_obj, corridor_obj, initTrajPlanner_obj, mission, param));
                resultPublisher_obj->plot(param.log);

                start_time = ros::Time::now().toSec();
                has_path = true;
            }
            
            // if path found and no new obstacles added
            if(has_path) {
                // Publish Swarm Trajectory
                current_time = ros::Time::now().toSec() - start_time;
                resultPublisher_obj.get()->update(current_time);
                resultPublisher_obj.get()->publish();
            }
        }
        else if (has_octomap && has_path) {

            // reset flags
            has_octomap = false;
            has_path = false;

            // update mission parameters
            mission.setMission_new(nh, current_x, current_y);
            param.setROSParam(nh);
            
            // print current values to confirm
            std::cout << "\nUpdated start positions: " << '\n';
            for(int a= 0; a<qn; a++)
                std::cout << a << " | x = " << mission.startState[a][0] << ", y = " << mission.startState[a][1] << '\n';

        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}