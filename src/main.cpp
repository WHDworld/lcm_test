#include "lcm_net.h"
#include <iostream>
void onTrajectoryData(const Trajectory_t &msg)
{
    std::cout << "Received trajectory data: " << msg.self_id << std::endl;
    std::cout << "Timestamp: " << msg.timestamp << std::endl;
    std::cout << "Self pose: " << msg.self_pose.position[0] << ", " << msg.self_pose.position[1] << ", " << msg.self_pose.position[2] << std::endl;
    std::cout << "Self velocity: " << msg.self_pose.velocity[0] << ", " << msg.self_pose.velocity[1] << ", " << msg.self_pose.velocity[2] << std::endl;
    std::cout << "Self acceleration: " << msg.self_pose.acceleration[0] << ", " << msg.self_pose.acceleration[1] << ", " << msg.self_pose.acceleration[2] << std::endl;
}

int main()
{
    std::string lcm_uri = "udpm://224.0.0.251:7667?ttl=1";
    int self_id = 0;
    LcmNet lcmnet_(lcm_uri, self_id);
    lcmnet_.trajectory_desc_callback = [&](const Trajectory_t &msg)
    {
        onTrajectoryData(msg);
    };
    while (true)
    {
        Trajectory_t traj;
        traj.self_id = self_id;
        traj.timestamp = 100;
        traj.self_pose.position[0] = 0;
        traj.self_pose.position[1] = 0;
        traj.self_pose.position[2] = 0;
        traj.self_pose.velocity[0] = 0;
        traj.self_pose.velocity[1] = 0;
        traj.self_pose.velocity[2] = 0;
        traj.self_pose.acceleration[0] = 0;
        traj.self_pose.acceleration[1] = 0;
        traj.self_pose.acceleration[2] = 0;
        traj.self_pose.orientation[0] = 0;
        traj.self_pose.orientation[1] = 0;
        traj.self_pose.orientation[2] = 0;
        traj.self_pose.orientation[3] = 0;
        lcmnet_.broadcastTrajectory(traj)
    }
}