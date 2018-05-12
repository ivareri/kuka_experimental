/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Norwegian University of Science and Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Norwegian University of Science and
 *     Technology, nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Lars Tingelstad <lars.tingelstad@ntnu.no>
 */

#include <kuka_rsi_hw_interface/kuka_hardware_interface.h>

#include <stdexcept>


namespace kuka_rsi_hw_interface
{

KukaHardwareInterface::KukaHardwareInterface() :
    joint_position_(12, 0.0), joint_velocity_(12, 0.0), joint_effort_(12, 0.0), joint_position_command_(12, 0.0), joint_velocity_command_(
        12, 0.0), joint_effort_command_(12, 0.0), joint_names_(12), rsi_initial_joint_positions_(12, 0.0), rsi_joint_position_corrections_(
        12, 0.0), rsi_tcp_position_corrections_(3, 0.0), ipoc_(0), n_dof_(6)
{
  in_buffer_.resize(1024);
  out_buffer_.resize(1024);
  remote_host_.resize(1024);
  remote_port_.resize(1024);

  nh_.param("rsi/external_axes", external_axes_, false);
  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "External axes: " << external_axes_);

  nh_.param("rsi/n_dof", n_dof_, 6);
  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "DOF: " << n_dof_);

  if (!nh_.getParam("controller_joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'controller_joint_names' "
      "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
      "'controller_joint_names' on the parameter server.");
  }


  if (nh_.getParam("rsi/force_torque_sensor_frame", force_torque_sensor_frame_))
  {
    ROS_INFO_STREAM_NAMED("hardware_interface", "Support for force torque sensor enabled");
    use_force_torque_sensor_ = true;
    nh_.param(std::string("rsi/force_torque_sensor_topic"), force_torque_sensor_topic_, std::string("ft_sensor/raw"));
    if (!nh_.hasParam("rsi/force_torque_sensor_topic"))
    {
      ROS_WARN("Cannot find parameter 'force_torque_sensor_topic' on the parameter server, using default "
             "'ft_sensor/raw'");
    }

  } else {
    ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "'force_torque_sensor_frame_' not on paramter server. Support for force torque sensor disabled");
    use_force_torque_sensor_ = false;
  }

  // Create ros_control interfaces
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    // Create joint state interface for all joints
    joint_state_interface_.registerHandle(
        hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i],
                                             &joint_effort_[i]));

    // Create joint position control interface
    position_joint_interface_.registerHandle(
        hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                        &joint_position_command_[i]));
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  // Create and register force torque sensor interface if enabled
  if (use_force_torque_sensor_) {
    force_torque_sensor_interface_.registerHandle(hardware_interface::ForceTorqueSensorHandle(
      force_torque_sensor_topic_, force_torque_sensor_frame_, force_, torque_));
   registerInterface(&force_torque_sensor_interface_);
  }


  ROS_INFO_STREAM_NAMED("hardware_interface", "Loaded kuka_rsi_hardware_interface");
}

KukaHardwareInterface::~KukaHardwareInterface()
{

}

bool KukaHardwareInterface::read(const ros::Time time, const ros::Duration period)
{
  in_buffer_.resize(1024);
  if (server_->recv(in_buffer_) == 0)
  {
    return false;
  }

  if (rt_rsi_recv_->trylock()){
    rt_rsi_recv_->msg_.data = in_buffer_;
    rt_rsi_recv_->unlockAndPublish();
  }

  rsi_state_ = RSIState(in_buffer_);

  // Update joint positions
  for (std::size_t i = 0; i < 6; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
  }

  for (std::size_t i = 6; i < n_dof_; ++i)
  {
    // Linear axes from KRC comes as [mm*RAD2DEG]
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i] / 1000;
  }

  // Update force and torque
  if (use_force_torque_sensor_) {
    for (std::size_t i = 0; i < 3; ++i)
    {
      force_[i] = rsi_state_.force[i];
      torque_[i] = rsi_state_.torque[i];
    }
  }

  // Update IPOC number
  ipoc_ = rsi_state_.ipoc;

  return true;
}

bool KukaHardwareInterface::write(const ros::Time time, const ros::Duration period)
{
  out_buffer_.resize(1024);

  for (std::size_t i = 0; i < 6; ++i)
  {
    rsi_joint_position_corrections_[i] = (RAD2DEG * joint_position_command_[i]) - rsi_initial_joint_positions_[i];
  }

  
  // With mathematically linked external axes, KRC will change robot joints to keep TCP static
  // Update TCP by same amount as linear axes to avoid this. Units in [mm]
  if (external_axes_) {

      // E1 & X
      rsi_joint_position_corrections_[6] = 1000 * (joint_position_command_[6] - rsi_initial_joint_positions_[6]);
      rsi_tcp_position_corrections_[0] = -rsi_joint_position_corrections_[6];

      // E2 & Y
      rsi_joint_position_corrections_[7] = 1000 * (joint_position_command_[7] - rsi_initial_joint_positions_[7]);
      rsi_tcp_position_corrections_[1] = -rsi_joint_position_corrections_[7];

      // E3 & Z
      rsi_joint_position_corrections_[8] = 1000 * (joint_position_command_[8] - rsi_initial_joint_positions_[8]);
      rsi_tcp_position_corrections_[2] = rsi_joint_position_corrections_[8];

  }
  
  out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_, rsi_tcp_position_corrections_, external_axes_).xml_doc;
  server_->send(out_buffer_);

  if(rt_rsi_send_->trylock()) {
    rt_rsi_send_->msg_.data = out_buffer_;
    rt_rsi_send_->unlockAndPublish();
  }
  return true;
}

void KukaHardwareInterface::start()
{
  // Wait for connection from robot
  server_.reset(new UDPServer(local_host_, local_port_));

  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Waiting for robot!");

  int bytes = server_->recv(in_buffer_);

  // Drop empty <rob> frame with RSI <= 2.3
  if (bytes < 100)
  {
    bytes = server_->recv(in_buffer_);
  }

  rsi_state_ = RSIState(in_buffer_);
  for (std::size_t i = 0; i < 6; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
    joint_position_command_[i] = joint_position_[i];
    rsi_initial_joint_positions_[i] = rsi_state_.initial_positions[i];
  }

  for (std::size_t i = 6; i < n_dof_; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i] / 1000;
    joint_position_command_[i] = joint_position_[i];

    // Linear external axes have different send and recevice units.
    // To KRC: [mm] From KRC: [mm * RAD2DEG]
    // Store initial position in [m]
    rsi_initial_joint_positions_[i] = joint_position_[i];
  }

  ipoc_ = rsi_state_.ipoc;
  out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_, rsi_tcp_position_corrections_, external_axes_).xml_doc;
  server_->send(out_buffer_);
  // Set receive timeout to 1 second
  server_->set_timeout(1000);
  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Got connection from robot");

}

void KukaHardwareInterface::configure()
{
  const std::string param_addr = "rsi/listen_address";
  const std::string param_port = "rsi/listen_port";

  if (nh_.getParam(param_addr, local_host_) && nh_.getParam(param_port, local_port_))
  {
    ROS_INFO_STREAM_NAMED("kuka_hardware_interface",
                          "Setting up RSI server on: (" << local_host_ << ", " << local_port_ << ")");
  }
  else
  {
    std::string msg = "Failed to get RSI listen address or listen port from"
    " parameter server (looking for '" + param_addr + "' and '" + param_port + "')";
    ROS_ERROR_STREAM(msg);
    throw std::runtime_error(msg);
  }

  rt_rsi_recv_.reset(new realtime_tools::RealtimePublisher<std_msgs::String>(nh_, "rsi_xml_doc_recv", 3));
  rt_rsi_send_.reset(new realtime_tools::RealtimePublisher<std_msgs::String>(nh_, "rsi_xml_doc_send", 3));
}

} // namespace kuka_rsi_hardware_interface
