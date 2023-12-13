#include <iostream>
#include <vector>
#include <signal.h>

// ROS headers
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>

// My Headers
#include "../include/final_project/PRM.h"
#include "../include/final_project/RRG.h"
#include "../include/final_project/PRMStar.h"
#include "../include/final_project/HelperClasses.h"

using nav_msgs::Odometry, nav_msgs::OccupancyGrid, nav_msgs::MapMetaData,
      nav_msgs::Path;
using geometry_msgs::Twist, geometry_msgs::Vector3, geometry_msgs::PoseStamped;

ros::Publisher path_pub;
ros::Publisher viz_pub;

float bot_circ = 0.2;
float bot_rad = bot_circ / 2.0; // meters
float bot_height = 0.3;

// 0 == PRM, 1 == PRM*, 2 == RRG 
#define ROADMAP_TYPE 1

#if ROADMAP_TYPE == 0
    PRM roadmap;
#elif ROADMAP_TYPE == 1
    PRMStar roadmap;
#else 
    RRG roadmap;
#endif

Vec2 goal_pos = Vec2(-3, 1);
Vec2 test_agent_pos = Vec2(0.0, 0.0);
Vec2 agent_pos(0, 0);
bool agent_at_goal = false;
bool path_updated = true;

std::vector<Circle> obstacles;

// movement stuff
float last_rotation = 0;
bool print_first_node = true;

bool in_odom = false;
bool in_scan = false;

// occupancy stuff
bool *dirty_arr;
bool dirty_arr_allocated = false;
vector<Circle> new_obstacles;
float filled_threshold = 0.98;
uint32_t dirty_counter = 0;
uint32_t gen_nodes_threshold = 100;

// path optimization
bool OPTIMIZE_PATH = false;
bool STATS_TESTING = false;
bool TEST_NODE_GEN = false;
bool TEST_PATH_GEN = false;
bool TEST_PATH_LEN = false;
bool TEST_PATH_QTY = false;
int NUM_TESTS = 10;
vector<int> NODE_NUMS = {
    10, 20, 50, 100, 1000
};
vector<int> OBSTACLE_NUMS = {
    0, 10, 100, 1000, 10000
};

nav_msgs::Path planned_path;

void testRandomizeRoadmap(int n_obsts) {
    roadmap.obstacles.clear();
    generateRandomObstacles(
        roadmap, n_obsts, 0.01, 0.01, vector<Vec2>{
            Vec2(-1, 1), Vec2(goal_pos.x - 1, goal_pos.x + 1)
        }
    );
    #if ROADMAP_TYPE == 0 || ROADMAP_TYPE == 1
        roadmap.generateNodes();
        roadmap.updateStartPos(test_agent_pos.x, test_agent_pos.y);
        roadmap.updateGoalPos(goal_pos.x, goal_pos.y);
        roadmap.connectStartAndGoal();
    #else
        roadmap.generateNodes(test_agent_pos, goal_pos);
    #endif
}

std::tuple<float, int> testPathQuality(int n_obst) {
    float total_quality = 0.0;
    int fails = 0;
    for (int i = 0; i < NUM_TESTS; i++) {
        testRandomizeRoadmap(n_obst);
        roadmap.updatePath();

        if (roadmap.path.size() == 0) {
            ++fails;
        } else if (roadmap.path.size() != 1) {
            Vec2 first_vec = test_agent_pos - roadmap.path[0];
            for (int j = 0; j < roadmap.path.size() - 1; j++) {
                // ratio of observed : actual
                if (j != 0) {
                    first_vec = roadmap.path[j-1] - roadmap.path[j];
                }
                Vec2 second_vec = roadmap.path[j+1] - roadmap.path[j];

                first_vec.normalize();
                second_vec.normalize();

                total_quality += std::acos(Vec2::dot(first_vec, second_vec));
            }
        }
    }

    total_quality *= TO_DEGS;
    
    if (fails != NUM_TESTS) 
        total_quality /= (NUM_TESTS - fails);

    return {total_quality, fails};
}

std::tuple<float, int> testPathLen(int n_obsts) {
    float total_len = 0.0;
    int fails = 0;
    float actual_len = (test_agent_pos - goal_pos).len();
    for (int i = 0; i < NUM_TESTS; i++) {
        testRandomizeRoadmap(n_obsts);
        roadmap.updatePath();

        if (roadmap.path.size() == 0) {
            ++fails;
        } else {
            total_len += (test_agent_pos - roadmap.path[0]).len();
            for (int j = 0; j < roadmap.path.size() - 1; j++) {
                // ratio of observed : actual
                total_len += (
                    roadmap.path[j] - roadmap.path[j+1]
                ).len() / actual_len;
            }
        }
    }
    
    if (NUM_TESTS != fails) 
        total_len /= (NUM_TESTS - fails);

    return {total_len, fails};
}

float testPathGen(int n_obsts) {
    float total_dur = 0.0;
    for (int i = 0; i < NUM_TESTS; i++) {
        testRandomizeRoadmap(n_obsts);
    
        total_dur += time_function("Generating Nodes", []() -> void {
            roadmap.updatePath();
        });
    }
    total_dur /= NUM_TESTS;
    return total_dur;
}

float testNodeGen(int n_obsts) {
    roadmap.obstacles.clear();
    generateRandomObstacles(
        roadmap, n_obsts, 0.0, 0.1, vector<Vec2>{Vec2(-1.5, 1.5)}
    );
    float total_dur = 0.0;
    for (int i = 0; i < NUM_TESTS; i++) {
        #if ROADMAP_TYPE == 0 || ROADMAP_TYPE == 1
            total_dur += time_function("Generating Nodes", []() -> void {
                roadmap.generateNodes();
                roadmap.updateStartPos(
                    test_agent_pos.x, test_agent_pos.y
                );
                roadmap.updateGoalPos(
                    goal_pos.x, goal_pos.y
                );
                roadmap.connectStartAndGoal();
            });
        #else
            total_dur += time_function("Generating Nodes", []() -> void {
                roadmap.generateNodes(test_agent_pos, goal_pos);
            });
        #endif
    }
    total_dur /= NUM_TESTS;
    return total_dur;
}

void mapCallback(const OccupancyGrid::ConstPtr& map) {
    auto &info = map->info;
    auto &grid = map->data;

    auto &position = info.origin.position;
    auto &orient = info.origin.orientation;

    if (!dirty_arr_allocated) {
        dirty_arr = new bool[info.width * info.height];
        dirty_arr_allocated = true;
    }
    float res = info.resolution;

    float half_res = res * 0.5;
    float obstacle_rad = bot_rad + half_res;
    bool generate_path = false;
    uint32_t x, y, i;
    for (i = 0, y = 0; y < info.height; ++y) {
        float cy = position.y + y * res + half_res;
        for (x = 0; x < info.width; ++x, ++i) {
            // 98% confidence that the square is filled
            if (!dirty_arr[i] && grid[i] > filled_threshold) {
                dirty_arr[i] = true;
                generate_path = true;

                ++dirty_counter;

                float cx = position.x + x * res + half_res;

                roadmap.obstacles.push_back(
                    Circle(Vec2(cx, cy), obstacle_rad)
                );
            }
        }
    }

    if (generate_path) {
        // update goal position
        if (dirty_counter > gen_nodes_threshold) {
            #if ROADMAP_TYPE == 0 || ROADMAP_TYPE == 1
                roadmap.generateNodes();
                roadmap.updateStartPos(agent_pos.x, agent_pos.y);
                roadmap.updateGoalPos(goal_pos.x, goal_pos.y);
                roadmap.connectStartAndGoal();
            #else
                roadmap.generateNodes(agent_pos, goal_pos);
            #endif
            dirty_counter = 0;
            // generate new path
            time_function("Updated Path", []() -> void {
                roadmap.updatePath();
            }, true);
        } else {
            time_function("Update Node Connections", []() -> void {
                roadmap.updateNodeConnections();
            });

            if (roadmap.regenerate_path) {
                #if ROADMAP_TYPE == 0 || ROADMAP_TYPE == 1
                    roadmap.generateNodes();
                    roadmap.updateStartPos(agent_pos.x, agent_pos.y);
                    roadmap.updateGoalPos(goal_pos.x, goal_pos.y);
                    roadmap.connectStartAndGoal();
                #else
                    roadmap.generateNodes(agent_pos, goal_pos);
                #endif
                time_function("Updated Path", []() -> void {
                    roadmap.updatePath();
                }, true);
                path_updated = true;
            }
        }
    }
}

void updateGoal(const Vector3::ConstPtr& msg) {
    goal_pos.x = msg->x;
    goal_pos.y = msg->y;
    agent_at_goal = false;
    #if ROADMAP_TYPE == 2
        roadmap.generateNodes(agent_pos, goal_pos);
    #endif
    // #if ROADMAP_TYPE == 0 || ROADMAP_TYPE == 1
    //     roadmap.updateStartPos(agent_pos.x, agent_pos.y);
    //     roadmap.updateGoalPos(goal_pos.x, goal_pos.y);
    //     roadmap.connectStartAndGoal();
    // #else
    //     
    // #endif
    // dirty_counter = 0;
    // generate new path
    // time_function("Updated Path", []() -> void {
    //     roadmap.updatePath();
    // }, true);
    // path_updated = true;
}

void odomCallback(const Odometry::ConstPtr& odom_msg) {
    // update agent position
    // agent to goal stuff
    agent_pos = Vec2(
        odom_msg->pose.pose.position.x, 
        odom_msg->pose.pose.position.y
    );
    Vec2 agent_to_goal = goal_pos - agent_pos;
    float atg_dist = agent_to_goal.len();

    // always done no matter the preprocessor directive
    if (roadmap.nodes.size() > 0) {
        roadmap.updateStartPos(agent_pos.x, agent_pos.y);
        #if ROADMAP_TYPE == 0 || ROADMAP_TYPE == 1
            roadmap.connectStart();
        #endif
    } 

    // if at goal, don't do anything
    if (atg_dist < TO_NODE_THRESH) {
        ROS_INFO("At Goal");
        agent_at_goal = true;
    }
}

// need this here to stop the bot from moving if ctrl+c in terminal
void sigintHandler(int sig) {
    planned_path.header.stamp = ros::Time::now();
    planned_path.header.frame_id = "robot_footprint";
    planned_path.poses.clear();
    path_pub.publish(planned_path);
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pathfinding", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    
    // srand((unsigned) time(NULL));

    nh.getParam("min", min);
    nh.getParam("max", max);

    c_world_bounds = BoundingBox(
        Vec2(min + bot_rad, min + bot_rad),
        Vec2(max - bot_rad, max - bot_rad)
    );

    // RRG Testing
    #if ROADMAP_TYPE == 0
        roadmap = PRM(
            c_world_bounds, // bounding box of world
            bot_rad, // radius of the turtlebot (simplified as circle again)
            150 // number of path nodes to generate
        );
        roadmap.generateNodes();
    #elif ROADMAP_TYPE == 1
        roadmap = PRMStar(c_world_bounds, bot_rad, 200);
        roadmap.generateNodes();
    #else 
        roadmap = RRG(c_world_bounds, bot_rad, 100);
        roadmap.generateNodes(agent_pos, goal_pos);
    #endif
    // gathering information about the 
    if (STATS_TESTING) {
        c_world_bounds = BoundingBox(Vec2(-10.0, -10.0), Vec2(10.0, 10.0));
        for (auto &n_nodes : NODE_NUMS) {
            #if ROADMAP_TYPE == 0
                roadmap = PRM(c_world_bounds, bot_rad, n_nodes);
            #elif ROADMAP_TYPE == 1
                roadmap = PRMStar(c_world_bounds, bot_rad, n_nodes);
            #else 
                roadmap = RRG(c_world_bounds, bot_rad, n_nodes);
            #endif 
            printf("n_nodes == %d\n", n_nodes);
            std::stringstream ss;
            for (auto &n_obsts : OBSTACLE_NUMS) {
                if (TEST_NODE_GEN)
                    ss << std::fixed << std::setprecision(3) << testNodeGen(n_obsts) << ";";
                if (TEST_PATH_GEN)
                    ss << std::fixed << std::setprecision(3) << testPathGen(n_obsts) << ";";
                if (TEST_PATH_LEN) {
                    auto [avg_ratio, fails] = testPathLen(n_obsts);
                    ss << std::fixed << std::setprecision(3) << avg_ratio << ", " << fails << ";";
                }
                if (TEST_PATH_QTY) {
                    auto [avg_qlty, fails] = testPathQuality(n_obsts);
                    ss << std::fixed << std::setprecision(3) << avg_qlty << ", " << fails << ";";
                }
            }
            printf("%s\n", ss.str().c_str());
        }
    } else {
        path_pub = nh.advertise<Path>("rm_path", 10);
        ros::Subscriber goal_sub = nh.subscribe("update_goal", 1, updateGoal);
        ros::Subscriber odom_sub = nh.subscribe("odom", 1, odomCallback);
        ros::Subscriber scan_sub = nh.subscribe("map", 1, mapCallback);

        ros::AsyncSpinner spinner(4);
        spinner.start();

        signal(SIGINT, sigintHandler);
        ros::Rate rate(1);
        while (ros::ok()) {
            planned_path.header.stamp = ros::Time::now();
            planned_path.header.frame_id = "robot_footprint";
            planned_path.poses.clear();

            cv::Mat img = cv::Mat::zeros(out_dim, out_dim, CV_8UC3);
            drawRoadmap(img, agent_pos, roadmap);

            char window[] = "Test Drawing";
            cv::imshow(window, img);
            cv::waitKey(1000);

            if (path_pub.getNumSubscribers() == 0) {
                rate.sleep();
                ROS_INFO("no subs");
                continue;
            }

            if (agent_at_goal) {
                ROS_INFO("Agent At Goal");
                path_pub.publish(planned_path);
                rate.sleep();
                continue;
            }

            #if ROADMAP_TYPE == 0 || ROADMAP_TYPE == 1
                roadmap.updateStartPos(agent_pos.x, agent_pos.y);
                roadmap.updateGoalPos(goal_pos.x, goal_pos.y);
                roadmap.connectStartAndGoal();
            // #else
            //     roadmap.generateNodes(agent_pos, goal_pos);
            #endif

            roadmap.updatePath();
            
            for (auto &node : roadmap.path) {
                PoseStamped pose;
                pose.pose.position.x = node.x;
                pose.pose.position.y = node.y;
                planned_path.poses.push_back(pose);
            }

            path_pub.publish(planned_path);

            // if no path, regenerate
            if (roadmap.path.size() == 0) {
                #if ROADMAP_TYPE == 0 || ROADMAP_TYPE == 1
                    time_function("Generate Nodes (after goal check)", []() -> void {
                        roadmap.generateNodes();
                    }, true);
                    roadmap.connectStartAndGoal();
                #else 
                    roadmap.generateNodes(agent_pos, goal_pos);
                #endif
                time_function("Update Path (after goal check)", []() -> void {
                    roadmap.updatePath();
                }, true);
                // if still no path, set twist to 0 ret, will regenerate on next odom call
                if (roadmap.path.size() == 0) {
                    ROS_INFO("No Path (After Goal Check)");
                    rate.sleep();
                    continue;
                }
            } else {
                path_updated = false;
            }

            rate.sleep();
        }
        
        planned_path.header.stamp = ros::Time::now();
        planned_path.header.frame_id = "robot_footprint";
        planned_path.poses.clear();
        path_pub.publish(nav_msgs::Path());

        if (dirty_arr_allocated) {
            delete[] dirty_arr;
        }
    }   

    return 0;
}
