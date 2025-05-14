#ifndef LCM_NET_H
#define LCM_NET_H

#include <ros/ros.h>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include <functional>
#include <mutex>
#include <thread>
#include <TrajectoryDescriptor_t.hpp>

inline ros::Time toROSTime(Time_t _time)
{
    return ros::Time(_time.sec, _time.nsec);
}

inline Time_t toLCMTime(ros::Time _time)
{
    Time_t t;
    t.sec = _time.sec;
    t.nsec = _time.nsec;
    return t;
}

struct Trajectory_t
{
    int self_id;
    double stamp;
    Eigen::Vector3d self_pose = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d self_vel = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d self_acc = Eigen::Vector3d(0, 0, 0);
    Eigen::Quaterniond self_attitude = Eigen::Quaterniond(1, 0, 0, 0);
    Trajectory_t() {}
    Trajectory_t(const TrajectoryDescriptor_t &traj_desc)
    {
        self_id = traj_desc.self_id;
        stamp = toROSTime(traj_desc.timestamp).toSec();
        self_pose.x() = traj_desc.self_pose.position[0];
        self_pose.y() = traj_desc.self_pose.position[1];
        self_pose.z() = traj_desc.self_pose.position[2];
        self_vel.x() = traj_desc.self_pose.velocity[0];
        self_vel.y() = traj_desc.self_pose.velocity[1];
        self_vel.z() = traj_desc.self_pose.velocity[2];
        self_acc.x() = traj_desc.self_pose.acceleration[0];
        self_acc.y() = traj_desc.self_pose.acceleration[1];
        self_acc.z() = traj_desc.self_pose.acceleration[2];
        self_attitude.x() = traj_desc.self_pose.orientation[0];
        self_attitude.y() = traj_desc.self_pose.orientation[1];
        self_attitude.z() = traj_desc.self_pose.orientation[2];
        self_attitude.w() = traj_desc.self_pose.orientation[3];
    }
    TrajectoryDescriptor_t toLCM() const
    {
        TrajectoryDescriptor_t ret;
        ret.self_id = self_id;
        ret.timestamp = toLCMTime(ros::Time(stamp));
        Pose_t self_pose_;
        self_pose_.position[0] = self_pose.x();
        self_pose_.position[1] = self_pose.y();
        self_pose_.position[2] = self_pose.z();
        self_pose_.velocity[0] = self_vel.x();
        self_pose_.velocity[1] = self_vel.y();
        self_pose_.velocity[2] = self_vel.z();
        self_pose_.acceleration[0] = self_acc.x();
        self_pose_.acceleration[1] = self_acc.y();
        self_pose_.acceleration[2] = self_acc.z();
        self_pose_.orientation[0] = self_attitude.x();
        self_pose_.orientation[1] = self_attitude.y();
        self_pose_.orientation[2] = self_attitude.z();
        self_pose_.orientation[3] = self_attitude.w();
        ret.self_pose = self_pose_;
        return ret;
    }
};

class LcmNet
{
    lcm::LCM lcm;
    int self_id;
    std::thread thread_process_lcm;
    std::recursive_mutex recv_lock;

public:
    std::function<void(const Trajectory_t &)> trajectory_desc_callback;

    LoopNet(std::string _lcm_uri, const int self_id_);

    int lcmHandle()
    {
        return lcm.handle();
    }
    bool print_network_status = true;
    void broadcastTrajectory(const Trajectory_t &trajectory);
    void onTrajectoryReceived(const lcm::ReceiveBuffer *rbuf,
                              const std::string &chan,
                              const TrajectoryDescriptor_t *msg);
};

#endif