#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include <math.h>
#include <string>

#include <vector>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "get_slam_results");


    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<hector_nav_msgs::GetRobotTrajectory>("trajectory");
    hector_nav_msgs::GetRobotTrajectory srv;
    float last_x = 0;
    float last_y = 0;
    float last_theta = 0;
    if (client.call(srv)) {
        std::cout << "frame_id,sec,nsec,fsec,x,y,ds,theta,dtheta" << std::endl;
        for(auto & pose : srv.response.trajectory.poses) {
            float x = pose.pose.position.x;
            float y = pose.pose.position.y;
            float dx = x-last_x;
            float dy = y-last_y;
            float ds = sqrt(dx*dx+dy*dy);
            float theta = atan2(dy,dx);
            float dtheta = theta-last_theta;
            
            cout 
                << pose.header.frame_id << "," 
                << pose.header.stamp.sec << "," 
                << pose.header.stamp.nsec << "," 
                << pose.header.stamp.sec << "." <<   pose.header.stamp.nsec << "," 
                << x << "," 
                << y << "," 
                << theta << "," 
                << ds << "," 
                << dtheta << endl;
            last_x = x;
            last_y = y;
            last_theta = theta;
        }
    }
   
    return 0;
}