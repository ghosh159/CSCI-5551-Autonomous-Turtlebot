#include <iostream>
#include <vector>
#include <signal.h>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <tf2/utils.h>
#include <tf/tf.h>

#include "../include/final_project/HelperClasses.h"

using namespace std::chrono;

using std::vector;
using nav_msgs::Path, nav_msgs::Odometry;
using geometry_msgs::Twist, geometry_msgs::PoseStamped, geometry_msgs::Vector3;

vector<Vec2> path;
Vec2 agent_pos(0, 0);
Vec2 goal_pos(0, 0);
float agent_rot = 0.0;
int path_node = 0;

ros::Publisher mov_pub;

bool OPTIMIZE_PATH = false;
bool TESTING = true;

auto time_to_goal_start = high_resolution_clock::now();
auto time_to_goal_end = high_resolution_clock::now();
duration<float> total_path_dur = duration<float>(0);
float total_distance_traveled = 0.0;
float optimal_total_path_length = 0.0;
Vec2 prev_agent_pos(0,0);
bool added_time = false;

void pathCallback(const Path::ConstPtr& path_msg) {
    path.clear();
    
    for (auto &pose : path_msg->poses)
        path.push_back(Vec2(pose.pose.position.x, pose.pose.position.y));

    path_node = 0;
}

void odomCallback(const Odometry::ConstPtr& odom_msg) {
    prev_agent_pos.x = agent_pos.x;
    prev_agent_pos.y = agent_pos.y;

    agent_pos.x = odom_msg->pose.pose.position.x;
    agent_pos.y = odom_msg->pose.pose.position.y;

    if (!added_time) {
        float len = (agent_pos - prev_agent_pos).len();
        total_distance_traveled += len;
    }

    tf2Scalar yaw, pitch, roll;
    tf2::Quaternion qt;
    tf2::convert(odom_msg->pose.pose.orientation, qt);
    tf2::getEulerYPR(qt, yaw, pitch, roll);

    agent_rot = yaw;
}

void goalCallback(const Vector3::ConstPtr& goal_msg) {
    printf("map");
    Vec2 new_goal(goal_msg->x, goal_msg->y);
    optimal_total_path_length += (goal_pos - new_goal).len();
    goal_pos.x = new_goal.x;
    goal_pos.y = new_goal.y;
    time_to_goal_start = high_resolution_clock::now();
    added_time = false;
    
}

// need this here to stop the bot from moving if ctrl+c in terminal
void sigintHandler(int sig) {
    mov_pub.publish(Twist());

    ROS_INFO("Shutting Down: %f %f %f", 
        total_path_dur.count(), 
        total_distance_traveled / optimal_total_path_length,
        optimal_total_path_length
    );

    ros::shutdown();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "path_follow", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    mov_pub = nh.advertise<Twist>("cmd_vel", 1);
    ros::Subscriber path_sub = nh.subscribe("rm_path", 1, pathCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odomCallback);
    ros::Subscriber goal_sub = nh.subscribe("update_goal", 1, goalCallback);
    ros::Rate rate(ros::Duration(0.1));

    signal(SIGINT, sigintHandler);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    while (ros::ok()) {
        if ((agent_pos - goal_pos).len() < TO_NODE_THRESH && !added_time) {
            time_to_goal_end = high_resolution_clock::now();
            duration<float> temp = (time_to_goal_end - time_to_goal_start);
            total_path_dur += (time_to_goal_end - time_to_goal_start);
            ROS_INFO("Intermediate: %f", temp.count());
            added_time = true;
        }

        if (path.size() == 0) {
            mov_pub.publish(Twist());
            rate.sleep();
            continue;
        }

        if (path_node > path.size()) {
            mov_pub.publish(Twist());
            rate.sleep();
            continue;
        }

        Vec2 &next_pos = path[path_node];
        // optimize the path
        // if (OPTIMIZE_PATH) {
        //     int path_size = path.size();
        //     int max_node_seen = 0;
        //     for (int i = 1; i < path_size; i++) {
        //         Vec2 ray = path[i] - agent_pos;
        //         float ray_dist = ray.len();
        //         ray.x /= ray_dist;
        //         ray.y /= ray_dist;
        //         if (!intersectsAnyCircle(
        //             roadmap.obstacles, agent_pos, ray_dist, ray
        //         )) {
        //             max_node_seen = i;
        //         }
        //     }

        //     for (int i = 0; i < max_node_seen; i++) {
        //         roadmap.path.erase(roadmap.path.begin());
        //     }

        //     next_pos = roadmap.path[0];
        // }
        
        Vec2 agent_to_next = next_pos - agent_pos;
        float atn_dist = agent_to_next.len();
        // at the end of this, next_pos should be the next node to go towards
        while (atn_dist < TO_NODE_THRESH) {
            // remove the node we are now at
            // path.erase(path.begin());
            // // make sure that the path isn't empty (it shouldnt be but)
            // if (path.size() == 0) {
            //     mov_pub.publish(Twist());
            //     rate.sleep();
            //     continue;
            // }
            // get next pos to head towards
            path_node++;
            next_pos = path[path_node];
            agent_to_next = next_pos - agent_pos;
            atn_dist = agent_to_next.len();
        }

        float goal_angle = std::atan2(
            next_pos.y - agent_pos.y, next_pos.x - agent_pos.x
        );

        float angle_diff = goal_angle - agent_rot;
        while (angle_diff > M_PI) 
            angle_diff -= MY_2PI;
        while (angle_diff < -M_PI)
            angle_diff += MY_2PI;

        // float temp = ANGULAR_SPEED * angle_diff;
        // twist.angular.z = temp > 0 ? std::min(temp, 1.5f) : std::max(temp, -1.5f);
        Twist twist;
        twist.linear.x = std::min(LINEAR_SPEED * atn_dist, (float)0.1);
        twist.angular.z = ANGULAR_SPEED * angle_diff;

        mov_pub.publish(twist);
        
        rate.sleep();
    }

    ROS_INFO("Shutting Down: %f %f", 
        total_path_dur.count(), total_distance_traveled 
    );

    mov_pub.publish(Twist());

    return 0;
}
