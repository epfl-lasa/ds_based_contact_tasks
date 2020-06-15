#ifndef __SURFACE_PUNCTURING_H__
#define __SURFACE_PUNCTURING_H__

#include <signal.h>
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
#include <deque>
#include <sstream>
#include "ros/ros.h"
#include <ros/package.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include <dynamic_reconfigure/server.h>
#include "Eigen/Eigen"
#include "CascadedBiquad.h"

#include <limbo/kernel/exp.hpp>
#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/data.hpp>
#include <limbo/mean/null_function.hpp>
#include <limbo/mean/constant.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/model/gp/kernel_lf_opt.hpp>
#include <limbo/tools.hpp>
#include <limbo/tools/macros.hpp>
#include <limbo/serialize/binary_archive.hpp>
#include <limbo/serialize/text_archive.hpp>
#include <limbo/kernel/matern_three_halves.hpp>
#include <limbo/kernel/matern_five_halves.hpp>


#define NB_FT_SAMPLES 50      			 // Number of force torque sensors' samples used for initial calibration (compute the offsets)
#define NB_TRACKED_OBJECTS 4         // Number of objects tracked by the motion capture system (optitrack)
#define MOVING_FORCE_WINDOW_SIZE 10  // Window's size used to average the force data and detect peristent contact
#define NB_ATTRACTORS 4




using namespace limbo;

struct Params {
    struct kernel_maternfivehalves {
        BO_PARAM(double, sigma_sq, 4.4883);
        BO_PARAM(double, l, 0.0051);
    };

    struct kernel_maternthreehalves {
        BO_PARAM(double, sigma_sq, 4.4883);
        BO_PARAM(double, l, 0.0051);
    };

    // struct mean_constant {
    //     ///@ingroup mean_defaults
    //     BO_PARAM(double, constant, 0.0f);
    // };
    struct kernel : public defaults::kernel {
        // BO_PARAM(bool, optimize_noise, true);
    };
    struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {
    };
    struct opt_rprop : public defaults::opt_rprop {
        // BO_PARAM(int, iterations, 20);
        // BO_PARAM(double, eps_stop, 1e-4);
    };
};

    using Kernel_32 = kernel::MaternThreeHalves<Params>;
    using Kernel_52 = kernel::MaternFiveHalves<Params>;
    using Mean_t = mean::NullFunction<Params>;
    using GP_32 = model::GP<Params, Kernel_32, Mean_t>;
    using GP_52 = model::GP<Params, Kernel_52, Mean_t>;


// the type of the GP
// using Kernel_t = kernel::SquaredExpARD<Params>;
// // using Mean_t = mean::Data<Params>;
// using Mean_t = mean::NullFunction<Params>;
// // using Mean_t = mean::Constant<Params>;
// using GP_t = model::GP<Params, Kernel_t, Mean_t>;//, model::gp::KernelLFOpt<Params>>;

class SurfacePuncturing 
{
	public:

    // Contact state:
    // CONTACT: The robot is in contact with the surface
    // CLOSE_TO_CONTACT: The robot is close to make contact with the surface
    // NO_CONTACT: The robot is not in contact with the surface
    enum ContactState {CONTACT = 0, CLOSE_TO_CONTACT = 1, NO_CONTACT = 2};


	private:
		///////////////////
		// ROS variables //
		///////////////////
		ros::NodeHandle _nh;  // Ros node handle
		ros::Rate _loopRate;  // Ros loop rate [Hz]
		float _dt;  					// Loop timestep [s]

		//////////////////////////////
		// Subscribers declarations //
		//////////////////////////////
		ros::Subscriber _subRobotPose;												  // Subscribe to robot pose
		ros::Subscriber _subRobotTwist;												  // Subscribe to robot twist
		ros::Subscriber _subForceTorqueSensor;								  // Subscribe to force torque sensor
		ros::Subscriber _subOptitrackPose[NB_TRACKED_OBJECTS];  // Subscribe to optitrack markers pose
		ros::Subscriber _subDampingMatrix;										  // Subscribe to damping matrix of DS-impedance controller

		///////////////////////////
		// Publisher declaration //
		///////////////////////////
		ros::Publisher _pubDesiredTwist;				// Publish desired twist to DS-impdedance controller
		ros::Publisher _pubDesiredOrientation;  // Publish desired orientation to DS-impedance controller
		ros::Publisher _pubFilteredWrench;			// Publish filtered measured wrench
		ros::Publisher _pubMarker;						  // Publish marker (RVIZ) 
		ros::Publisher _pubTaskAttractor;				// Publish attractor on surface (RVIZ)
		ros::Publisher _pubNormalForce;					// Publish measured normal force to the surface
		ros::Publisher _pubDepth;					// Publish measured normal force to the surface
		
		//////////////////////////
		// Messages declaration //
		//////////////////////////
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		geometry_msgs::WrenchStamped _msgFilteredWrench;
		
		//////////////////////////
		// Tool characteristics //
		//////////////////////////
		float _toolMass;														 // Tool mass [kg]
		Eigen::Vector3f _toolOffsetFromEE;										 // Tool offset along z axis of end effector [m]							
		Eigen::Vector3f _toolComPositionFromSensor;  // Offset of the tool [m]	(3x1)
		Eigen::Vector3f _gravity;										 // Gravity vector [m/s^2] (3x1)

		//////////////////////////
		// Tool state variables //
		//////////////////////////
		Eigen::Vector3f _x;												 // Position [m] (3x1)
		Eigen::Vector4f _q;												 // Quaternion (4x1)
		Eigen::Matrix3f _wRb;											 // Orientation matrix (3x1) (form end effector to world frame)
		Eigen::Vector3f _v;												 // Velocity [m/s] (3x1)
		Eigen::Vector3f _w;												 // Angular velocity [rad/s] (3x1)
		Eigen::Matrix<float,6,1> _wrench;					 // Wrench [N and Nm] (6x1)
		Eigen::Matrix<float,6,1> _wrenchBias;			 // Wrench bias [N and Nm] (6x1)
		Eigen::Matrix<float,6,1> _filteredWrench;	 // Filtered wrench [N and Nm] (6x1)
		float _prevNormalForce;	 // Filtered wrench [N and Nm] (6x1)
		float _prevDNormalForce;	 // Filtered wrench [N and Nm] (6x1)
    float _normalForce;												 // Normal force to the surface [N]
    float _dNormalForce;												 // Normal force to the surface [N]
    float _ddNormalForce;												 // Normal force to the surface [N]
    float _prevDDNormalForce;												 // Normal force to the surface [N]
    std::vector<float> _positivePicDForce;
    std::vector<float> _positivePicTime;
    std::vector<float> _negativePicDForce;
    std::vector<float> _negativePicTime;
    bool _picDetected;
		int _wrenchCount;													 // Counter used to pre-process the force data
    float _normalDistance;										 // Normal distance to the surface [m]
		Eigen::Matrix3f _D;                        // Damping matrix used by the DS-impedance controller (3x3)
		float _d1;                                 // Damping gain along the desired velocity direction (DS-impedance controller)
    ContactState _contactState;                // Contact state with the surface
		float _c;                 								 // Contact value (1 = CONTACT, 0 otherwise)

    ////////////////////////////
		// Tool control variables //
		////////////////////////////
		Eigen::Vector3f _xd;			// Desired position [m] (3x1)
		Eigen::Vector3f _xc;			// Desired position [m] (3x1)
		Eigen::Vector4f _qd;			// Desired quaternion (4x1)
		Eigen::Vector3f _omegad;	// Desired angular velocity [rad/s] (3x1)
		Eigen::Vector3f _fx;			// Nominal DS [m/s] (3x1)
		Eigen::Vector3f _fxc;			// Desired conservative part of the nominal DS [m/s] (3x1)
		Eigen::Vector3f _fxr;			// Desired non-conservative part of the nominal DS [m/s] (3x1)
		Eigen::Vector3f _fxn;			// Modulation velocity term along normal direction [m/s] (3x1)
		Eigen::Vector3f _fxp;			// Corrected nominal DS to ensure passivity [m/s] (3x1)
		Eigen::Vector3f _fxnp;		// Corrected modulation term along normal direction to ensure passivity [m/s] (3x1)
		Eigen::Vector3f _vd;			// Desired modulated DS [m/s] (3x1)
		float _targetForce;				// Target force in contact [N]
		float _targetVelocity;		// Velocity norm of the nominal DS [m/s]
		float _Fd;								// Desired force profile [N]

		////////////////////
		// Task variables //
		////////////////////
		int _attractorID;
    Eigen::MatrixXf _xa;	 // Attractor position [m] (3x1)
    Eigen::Vector3f _planeNormal;		 			 // Normal vector to the surface (pointing outside the surface) (3x1)
    Eigen::Vector3f _n;							  		 // Normal vector to the surface (pointing towards the surface) (3x1)
    Eigen::Vector3f _xProj;					 			 // Vertical projection on the surface [m] (3x1)
    Eigen::Vector3f _xs;					 			 // Vertical projection on the surface [m] (3x1)
    Eigen::Matrix3f _wRs;						 			 // Orientation matrix from surface frame to world frame
		std::deque<float> _normalForceWindow;  // Moving window saving the robot's measured normal force to the object's surface [N]
		float _normalForceAverage;						 // Average normal force measured through the force windows [N]
		float _normalForceTolerance;           // Normal force tolerance to detect contactc [N]
  	float _normalDistanceTolerance;        // Normal distance tolerance to detect contact [m]
  	float _depth;
  	float _ke;
  	float _deltac;
  	float _deltad;
  	float _attractorOffset;
  	float _puncturingVelocityLimit;


    //////////////
    // Booleans //
    //////////////
		bool _firstRobotPose;													// Monitor the first robot pose update
		bool _firstRobotTwist;												// Monitor the first robot twist update
		bool _firstWrenchReceived;										// Monitor first force/torque data update
		bool _firstDampingMatrix;											// Monitor first damping matrix update
		bool _wrenchBiasOK;														// Check if computation of force/torque sensor bias is OK
		bool _stop;																		// Check for CTRL+C
		bool _sim;

		////////////////////
		// User variables //
		////////////////////
		float _velocityLimit;			 // Velocity limit [m/s]
		float _filteredForceGain;	 // Filtering gain for force/torque sensor
    Eigen::Vector3f _offset;	 // Attractor offset on surface [m] (3x1)
    double _duration;					 // Duration of an experiment [s]
		
		/////////////////////
		// Other variables //
		/////////////////////
    uint32_t _nbContact;
    double _timeInit;
		std::string _fileName;  			// Filename used to log data
		std::ifstream _inputFile;  		// Input stream object to read a text file
		std::ofstream _outputFile;  	// Output stream object to right to a text file
		std::mutex _mutex;  					// Mutex variable
		static SurfacePuncturing* me;  // Pointer on the instance of the class

		float _deltav = 0.0f;

		std::deque<Eigen::Vector3f> _rawForces, _filteredForces;
		CascadedBiquad _filter;

		GP_32 *_gp[3];
		// GP_32 *_gp2, *_gp3;
		std::deque<float> _gpForce[3];
		std::deque<float> _forceWindow;

		int _idModel;
	// GP_t *_gp;
		// GP_t *_gp;

	float _compensation;
	bool _useModel;
	float _adaptationRate;	
	bool _useAdaptation;
	Eigen::Vector3f _forceModel;

	public:

		// Class constructor
		SurfacePuncturing(ros::NodeHandle &n, double frequency, std::string fileName);

		// Initialize node
		bool init();

		// Run node
		void run();

	private:

		// Callback called when CTRL is detected to stop the node
		static void stopNode(int sig);

		bool allSubscribersOK();

		// Compute command to be sent to the DS-impedance controller
    void computeCommand();

    // Update surface info (normal vector and distance)
		void updateSurfaceInformation();

		// Update contact state with the surface
		void updateContactState();

		// Compute nominal DS
		void computeNominalDS();

		// Compute desired contact force profile
		void computeDesiredContactForceProfile();
		
		// Compute modulation terms along tangential and normal direction to the surface
		void computeModulationTerms();

		// Compute modulated DS (desired velocity)
		void computeModulatedDS();

		// Compute desired orientation
		void computeDesiredOrientation();
    
  	// Log data to text file
    void logData();

    // Publish data to topics
    void publishData();

    // Callback to update the robot pose
    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg);

    // Callback to update the robot twist
    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg);

    // Callback to update the robot wrench (force/torque sensor data)
    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg);

		// Callback to update damping matrix form the DS-impedance controller
    void updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg); 
};


#endif
