#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>
#include <cmath>

namespace gazebo
{
  class VelControlPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VelControlPlugin() {}

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr rev_joint, prismat_joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    private: event::ConnectionPtr updateConnection;

    // Default to zero velocity
    private: double rev_velocity = 1.0;

    private: double prismat_velocity = 51.0;

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, VelControl plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      this->rev_joint = _model->GetJoint("rev");
      this->prismat_joint = _model->GetJoint("prismat");

      // Setup a P-controller, with a gain of 1.
      this->pid = common::PID(1, 0, 0);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(this->rev_joint->GetScopedName(), this->pid);
      this->model->GetJointController()->SetVelocityPID(this->prismat_joint->GetScopedName(), this->pid);

      // Check that the velocity element exists, then read the value
      if (_sdf->HasElement("rev_velocity") && _sdf->HasElement("prismat_velocity")){
        this->rev_velocity = _sdf->Get<double>("rev_velocity");
        this->prismat_velocity = _sdf->Get<double>("prismat_velocity");
      }

      // Create the node
      this->node = transport::NodePtr(new transport::Node());

      this->node->Init(this->model->GetWorld()->Name());

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&VelControlPlugin::UpdateStates, this));
 
      }

    /// \brief Set the velocity of the VelControl
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double &_velrev, const double &_velprismat)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(this->rev_joint->GetScopedName(), _velrev);
      this->model->GetJointController()->SetVelocityTarget(this->prismat_joint->GetScopedName(), _velprismat);
    }

    void UpdateStates(){

        static double t=0;

        static double p = 50;

        if(t > 4.999 && t < 5.002){
            p += 0.5;
        }else if(t > 10){
            p -= 3*0.5;
            t -= t;
        }
        t = t + 0.001;
        
        // this->SetVelocity(this->rev_velocity, this->prismat_velocity);
        this->SetVelocity(this->rev_velocity, p);

    }

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelControlPlugin)
}
