/** COPYRIGHT
 * http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5
 **/

#ifndef VELOCITY_CONTROL_VELODYNE_GAZEBO_PLUGIN_HH_
#define VELOCITY_CONTROL_VELODYNE_GAZEBO_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"


namespace gazebo {


class VelocityControlVelodyne:public ModelPlugin {
 private:
  physics::ModelPtr model_;
  physics::JointPtr joint_;

  double velocity_;
  common::PID pid_;

  transport::NodePtr node_;
  transport::SubscriberPtr velSub_;
  transport::PublisherPtr posePub_;

  std::unique_ptr<ros::NodeHandle> rosNode_;
  ros::Subscriber rosVelSub_;
  ros::CallbackQueue rosQueue_;
  std::thread rosQueueThread_;

  void SetVelocity();
  void QueueThread();

 public:
  VelocityControlVelodyne() {}
  ~VelocityControlVelodyne() {}

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnVelMsg(ConstVector3dPtr &_msg);
  void OnRosVelMsg(const std_msgs::Float32ConstPtr &_msg);
};

void VelocityControlVelodyne::Load(physics::ModelPtr _model,
                                         sdf::ElementPtr _sdf) {

  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init();

  transport::PublisherPtr posePub_ =
    node_->Advertise<msgs::PosesStamped>("~/pose_example", 1, 100);

  transport::SubscriberPtr velSub_ =
    node_->Subscribe("~/vel_cmd",
                    &VelocityControlVelodyne::OnVelMsg,
                    this);

  if (_model->GetJointCount() == 0) {
    std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
    return;
  }

  this->model_ = _model;
  this->joint_ = _model->GetJoints()[0];
  this->pid_ = common::PID(0.1, 0, 0);
  this->model_->GetJointController()->SetVelocityPID(
      this->joint_->GetScopedName(), this->pid_);

  velocity_ = 0.0;
  if (_sdf->HasElement("velocity"))
    velocity_ = _sdf->Get<double>("velocity");
  else
    std::cout << "No velocity on gazebo.world";



  this->SetVelocity();

  // ROS Init topic
  if (!ros::isInitialized()) {
    int argc {0};
    char **argv = nullptr;
    ros::init(argc, argv, "gazebo_client",
      ros::init_options::NoSigintHandler);
  }

  this->rosNode_.reset(new ros::NodeHandle("gazebo_client"));


  ROS_INFO_STREAM("teste rosssss");

  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<std_msgs::Float32>(
      "/" + this->model_->GetName() + "/vel_cmd",
      1,
      boost::bind(&VelocityControlVelodyne::OnRosVelMsg, this, _1),
      ros::VoidPtr(), &this->rosQueue_);
  this->rosVelSub_ = this-> rosNode_->subscribe(so);

  this->rosQueueThread_ =
    std::thread(std::bind(&VelocityControlVelodyne::rosQueueThread_,
      this));
  std::cout << "Teste!";
}

void VelocityControlVelodyne::OnVelMsg(ConstVector3dPtr &_msg) {
  this->velocity_ = _msg->x();
  this->SetVelocity();
  std::cout << "OnVelMsg vel_:" << velocity_;
}

void VelocityControlVelodyne::SetVelocity() {
  this->model_->GetJointController()->SetVelocityTarget(
    this->joint_->GetScopedName(), this->velocity_);
  std::cout << "SetVelocity vel_:" <<
    this->model_->GetJointController()->GetVelocities()[0];
}

void VelocityControlVelodyne::OnRosVelMsg(
  const std_msgs::Float32ConstPtr &_msg) {
  this->velocity_ = _msg->data;
  this->SetVelocity();
  std::cout << "OnRosVelMsg vel_:" << velocity_;
}

void VelocityControlVelodyne::QueueThread() {
  static const double timeout {0.01};
  while (this->rosNode_->ok()) {
    this->rosQueue_.callAvailable(ros::WallDuration(timeout));
  }

}


  GZ_REGISTER_MODEL_PLUGIN(VelocityControlVelodyne)

}  // namespace gazebo
#endif  // VELOCITY_CONTROL_VELODYNE_GAZEBO_PLUGIN_HH_
