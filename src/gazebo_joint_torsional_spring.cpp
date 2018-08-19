#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>

namespace gazebo
{
	class TorsionalSpringPlugin : public ModelPlugin
	{
		public: TorsionalSpringPlugin() {
			ROS_INFO("Loaded gazebo_joint_torsional_spring.");
		}

		// Pointer to model
		private: physics::ModelPtr model;

		// Pointer to joint
		private: physics::JointPtr joint;

		// Spring constant
		private: double setPoint;

		// Rest angle
		private: double kx;

		// Pointer to update event connection
		private: event::ConnectionPtr updateConnection;

		// Load is called by Gazebo when the plugin is inserted into simulation
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			// Safety check
			if (_model->GetJointCount() == 0)
			{
				std::cerr << "You have zero joints! Something is wrong! Not loading plugin.\n";
				return;
			}

			// Store model pointer
			this->model = _model;

			if (_sdf->HasElement("joint"))
			{
				this->joint = _model->GetJoint(_sdf->Get<std::string>("joint"));
			}

			if (_sdf->HasElement("k"))
				this->kx = _sdf->Get<double>("k");

			if (_sdf->HasElement("set_point"))
				this->setPoint = _sdf->Get<double>("set_point");

			// // Store the joints you want to have a torsional spring applied
			// this->joint = _model->GetJoint("LEFT_KNEE");

			// Listen to update event
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind (&TorsionalSpringPlugin::OnUpdate, this) );
		}

		;

		public: void OnUpdate()
		{
			// this->setPoint = 0;
			// this->kx = 0.000139;

			double current_angle = this->joint->GetAngle(0).Radian();
			this->joint->SetForce(0, this->kx*(this->setPoint-current_angle));
		}
	};

	// Register the plugin such that Gazebo can call Load on this plugin
	GZ_REGISTER_MODEL_PLUGIN(TorsionalSpringPlugin)
}