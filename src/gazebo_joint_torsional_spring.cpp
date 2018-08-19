#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// #include "gazebo/common/Assert.hh"
// #include <gazebo/common/Event.hh>
#include <gazebo/common/common.hh>
// #include <gazebo/common/Plugin.hh>

// #include <ros/ros.h>

namespace gazebo
{
	class TorsionalSpringPlugin : public ModelPlugin
	{
		public: TorsionalSpringPlugin() {}

		
		private:
			// Pointer to model
			physics::ModelPtr model;
			// Pointer to SDF model
			sdf::ElementPtr sdf;
			// Pointer to joint
			
			physics::JointPtr joint;
			// Set point
			double setPoint;
			// Spring constant
			double kx;
			// Pointer to update event connection
			event::ConnectionPtr updateConnection;
			
			// Node handle
			// std::unique_ptr<ros::NodeHandle> rosNode;



		// Load is called by Gazebo when the plugin is inserted into simulation
		public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			// if (!ros::isInitialized())
			// {
			// 	int argc = 0;
   //              char **argv = NULL;
   //              ros::init(argc, argv, "spring_joint", ros::init_options::NoSigintHandler);
			// }
		// 	GZ_ASSERT(_model != NULL, "Received NULL model pointer for JointTorsionalSping Plugin!")

		// 	GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer for JointTorsionalSping Plugin!")

			// Safety check
			if (_model->GetJointCount() == 0)
			{
				std::cerr << "You have zero joints! Something is wrong! Not loading plugin.\n";
				return;
			}

			// this->rosNode.reset(new ros::NodeHandle("spring_joint"));

			// Store model pointer
			this->model = _model;

			// Store the SDF pointer
			this->sdf = _sdf;

			if (_sdf->HasElement("joint"))
				this->joint = _model->GetJoint(_sdf->Get<std::string>("joint"));
			else
				std::cout << "Must specify joint to apply a torsional spring at!\n";

			this->kx = 0.0;
			if (_sdf->HasElement("kx"))
				this->kx = _sdf->Get<double>("kx");
			else
				printf("Torsional spring coefficient not specified! Defaulting to: %f\n", this->kx);

			this->setPoint = 0.0;
			if (_sdf->HasElement("set_point"))
				this->setPoint = _sdf->Get<double>("set_point");
			else
				printf("Set point not specified! Defaulting to: %f\n", this->setPoint);

			std::cout << "Loaded gazebo_joint_torsional_spring.\n";

		}

		
		public: void Init()
		{
			// Listen to update event
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind (&TorsionalSpringPlugin::OnUpdate, this) );
		}

		protected: void OnUpdate()
		{
			double current_angle = this->joint->GetAngle(0).Radian();
			this->joint->SetForce(0, this->kx*(this->setPoint-current_angle));			
		}
		
	};

	// Register the plugin such that Gazebo can call Load on this plugin
	GZ_REGISTER_MODEL_PLUGIN(TorsionalSpringPlugin)
}