#include <lcm_net.h>

LcmNet::LcmNet(std::string _lcm_uri, const int self_id_) : lcm(_lcm_uri), self_id(self_id_), recv_period(_recv_period)
{
    if (!lcm.good())
    {
        ROS_ERROR("LCM %s failed", _lcm_uri.c_str());
        exit(-1);
    }
    lcm.subscribe("DRONE_TRAJECTORY", &LoopNet::onTrajectoryReceived, this);

    this->thread_process_lcm = std::thread([&]
                                           {
            printf("[thread_process_lcm] Start thread_process_lcm.\n");
            while (0 == this->lcmHandle())
            {
            }
            printf("[thread_process_lcm] thread_process_lcm exit.\n"); });
}

void LcmNet::broadcastTrajectory(const Trajectory_t &trajectory)
{
    TrajectoryDescriptor_t traj_desc = trajectory.toLCM();
    lcm.publish("DRONE_TRAJECTORY", &traj_desc);
    if (this->print_network_status)
    {
        int byte_sent = traj_desc.getEncodedSize();
        ROS_DEBUG("[SWARM_LOOP] uav %d Broadcast Trajectory. sum size: %d Byte", traj_desc.self_drone_id, byte_sent);
    }
}

void LcmNet::onTrajectoryReceived(const lcm::ReceiveBuffer *rbuf,
                                  const std::string &chan,
                                  const TrajectoryDescriptor_t *msg)
{
    std::lock_guard<std::recursive_mutex> Guard(recv_lock);
    if (msg->self_drone_id != this->self_id)
    {
        trajectory_desc_callback(*msg);
    }
}