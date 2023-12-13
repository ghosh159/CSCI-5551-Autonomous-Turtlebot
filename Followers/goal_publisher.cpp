#include <iostream>
#include <signal.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

using geometry_msgs::Vector3;

// static bool end = false;
// void sigintHandler(int sig) {
//     end = true;
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_publish"/*, ros::init_options::NoSigintHandler*/);

    ros::NodeHandle nh;
    ros::Publisher goal_pub = nh.advertise<Vector3>("update_goal", 1);

    // signal(SIGINT, sigintHandler);

    while (ros::ok()) {
        float x, y;
        printf("Input Goal Position: x y\n");
        std::cin >> x; std::cin >> y;
        
        Vector3 vec_msg;
        vec_msg.x = x;
        vec_msg.y = y;

        goal_pub.publish(vec_msg);

        ros::spinOnce();
    }

    return 0;
}
