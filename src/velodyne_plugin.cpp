/** COPYRIGHT
 * http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5
 **/

#ifndef VELODYNE_PLUGIN_HH_
#define VELODYNE_PLUGIN_HH_

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
  /// \brief A plugin to control a Velodyne sensor.
class VelodynePlugin:public ModelPlugin {
 private:
  // \brief Pointer to the model.
  physics::ModelPtr model;
  /// \brief Pointer to the joint.
  physics::JointPtr joint;



  /// \brief A PID controller for the joint.
  common::PID pid;
  /// \brief A node used for transport
  transport::NodePtr node;
  /// \brief A subscriber to a named topic.
  transport::SubscriberPtr sub;
  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> rosNode;
  /// \brief A ROS subscriber
  ros::Subscriber rosSub;
  /// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;
  /// \brief A thread the keeps running the rosQueue
  std::thread rosQueueThread;

  /// \brief ROS helper function that processes messages
  void QueueThread();

 public:
  /// \brief Constructor
  VelodynePlugin() {}

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Set the velocity of the Velodyne
  /// \param[in] _vel New target velocity
  void SetVelocity(const double &_vel);

  /// \brief Handle incoming message
  /// \param[in] _msg Repurpose a vector3 message. This function will
  /// only use the x component.
  void OnMsg(ConstVector3dPtr &_msg);

  /// \brief Handle an incoming message from ROS
  /// \param[in] _msg A float value that is used to set the velocity
  /// of the Velodyne.
  void OnRosMsg(const std_msgs::Float32ConstPtr &_msg);
};

void VelodynePlugin::Load(physics::ModelPtr _model,
                          sdf::ElementPtr _sdf) {
  // Safety check
  if (_model->GetJointCount() == 0) {
    std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
    return;
  }

  this->model = _model;
  this->joint = _model->GetJoints()[0];
  this->pid = common::PID(0.1, 0, 0);
  this->model->GetJointController()->SetVelocityPID(
      this->joint->GetScopedName(), this->pid);

  double velocity = 0;

  if (_sdf->HasElement("velocity"))
    velocity = _sdf->Get<double>("velocity");

  this->SetVelocity(velocity);

  // Create the node
  this->node = transport::NodePtr(new transport::Node());
  #if GAZEBO_MAJOR_VERSION < 8
  this->node->Init(this->model->GetWorld()->GetName());
  #else
  this->node->Init(this->model->GetWorld()->Name());
  #endif

  // Create a topic name
  std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

  // Subscribe to the topic, and register a callback
  this->sub = this->node->Subscribe(topicName,
    &VelodynePlugin::OnMsg, this);

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<std_msgs::Float32>(
        "/" + this->model->GetName() + "/vel_cmd",
        1,
        boost::bind(&VelodynePlugin::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
  this->rosSub = this->rosNode->subscribe(so);

  // Spin up the queue helper thread.
  this->rosQueueThread =
    std::thread(std::bind(&VelodynePlugin::QueueThread, this));
}

void VelodynePlugin::SetVelocity(const double &_vel) {
  // Set the joint's target velocity.
  this->model->GetJointController()->SetVelocityTarget(
      this->joint->GetScopedName(), _vel);
}

void VelodynePlugin::OnMsg(ConstVector3dPtr &_msg) {
  this->SetVelocity(_msg->x());
}

/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the Velodyne.
void VelodynePlugin::OnRosMsg(const std_msgs::Float32ConstPtr &_msg) {
  this->SetVelocity(_msg->data);
}

/// \brief ROS helper function that processes messages
void VelodynePlugin::QueueThread() {
  static const double timeout = 0.01;
  while (this->rosNode->ok()) {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)


}  // namespace gazebo
#endif  // VELODYNE_PLUGIN_HH_
