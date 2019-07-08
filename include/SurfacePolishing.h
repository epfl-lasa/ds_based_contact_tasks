#ifndef __SURFACE_POLISHING_H__
#define __SURFACE_POLISHING_H__

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
#include "ds_based_contact_tasks/surfacePolishing_paramsConfig.h"
#include "Eigen/Eigen"
#include "svm_grad.h"
#include "adaptation_rbf.hpp"
#include "RbfAdaptation.hpp"


#define NB_FT_SAMPLES 50      			 // Number of force torque sensors' samples used for initial calibration (compute the offsets)
#define NB_OPTITRACK_SAMPLES 100     // Number of optitrack samples used for initial objects' pose estimation
#define NB_TRACKED_OBJECTS 4         // Number of objects tracked by the motion capture system (optitrack)
#define MOVING_FORCE_WINDOW_SIZE 10  // Window's size used to average the force data and detect peristent contact

class SurfacePolishing 
{
	public:
		// Surface type (planar or non flat)
		enum SurfaceType {PLANAR = 0, NON_FLAT = 1};

    // Contact state:
    // CONTACT: The robot is in contact with the surface
    // CLOSE_TO_CONTACT: The robot is close to make contact with the surface
    // NO_CONTACT: The robot is not in contact with the surface
    enum ContactState {CONTACT = 0, CLOSE_TO_CONTACT = 1, NO_CONTACT = 2};

    // Optitrack object ID:
    // ROBOT_BASIS Markers placed at the basis of the robot
    // P{i=1,..3} Markers placed on the target surface 	
    enum ObjectID {ROBOT_BASIS = 0, P1 = 1, P2 = 2, P3 = 3};

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
		
		//////////////////////////
		// Messages declaration //
		//////////////////////////
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		visualization_msgs::Marker _msgSurfaceMarker;
		visualization_msgs::Marker _msgArrowMarker;
		geometry_msgs::PointStamped _msgTaskAttractor;
		geometry_msgs::WrenchStamped _msgFilteredWrench;
		
		//////////////////////////
		// Tool characteristics //
		//////////////////////////
		float _toolMass;														 // Tool mass [kg]
		float _toolOffsetFromEE;										 // Tool offset along z axis of end effector [m]							
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
    float _normalForce;												 // Normal force to the surface [N]
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
		Eigen::Vector4f _qd;			// Desired quaternion (4x1)
		Eigen::Vector3f _omegad;	// Desired angular velocity [rad/s] (3x1)
		Eigen::Vector3f _fx;			// Nominal DS [m/s] (3x1)
		Eigen::Vector3f _fxc;			// Desired conservative part of the nominal DS [m/s] (3x1)
		Eigen::Vector3f _fxr;			// Desired non-conservative part of the nominal DS [m/s] (3x1)
		Eigen::Vector3f _fxn;			// Modulation velocity term along normal direction [m/s] (3x1)
		Eigen::Vector3f _fxm;			// Modulation velocity term correcting the robot motion [m/s] (3x1)
		Eigen::Vector3f _fxp;			// Corrected nominal DS to ensure passivity [m/s] (3x1)
		Eigen::Vector3f _fxnp;		// Corrected modulation term along normal direction to ensure passivity [m/s] (3x1)
		Eigen::Vector3f _vd;			// Desired modulated DS [m/s] (3x1)
		float _targetForce;				// Target force in contact [N]
		float _targetVelocity;		// Velocity norm of the nominal DS [m/s]
		float _Fd;								// Desired force profile [N]
		float _Fdp;								// Corrected desired force profile to ensure passivity [N]

		////////////////////
		// Task variables //
		////////////////////
		SurfaceType _surfaceType;				 			 // Surface type
		SVMGrad _svm;                          // Instance of the SVMGrad class used to evaluate the learned surface state (distance+normal vector)
    Eigen::Vector3f _polishingAttractor;	 // Attractor position [m] (3x1)
    Eigen::Vector3f _planeNormal;		 			 // Normal vector to the surface (pointing outside the surface) (3x1)
    Eigen::Vector3f _n;							  		 // Normal vector to the surface (pointing towards the surface) (3x1)
    Eigen::Vector3f _xProj;					 			 // Vertical projection on the surface [m] (3x1)
    Eigen::Matrix3f _wRs;						 			 // Orientation matrix from surface frame to world frame
		std::deque<float> _normalForceWindow;  // Moving window saving the robot's measured normal force to the object's surface [N]
		float _normalForceAverage;						 // Average normal force measured through the force windows [N]
		float _normalForceTolerance;           // Normal force tolerance to detect contactc [N]
  	float _normalDistanceTolerance;        // Normal distance tolerance to detect contact [m]

    /////////////////////////////////////////////
    // Normal modulation adaptation parameters //
    /////////////////////////////////////////////
  	float _deltaF;
  	float _epsilonF;
  	float _epsilonF0;
  	float _gammaF;
  	float _deltaf;
  	float _epsilonf;
  	float _epsilonf0;
  	float _gammaf;

		///////////////////////
		// Tank's parameters //
		///////////////////////
		float _s;				// Current tank level
		float _smax;		// Max tank level
		float _alpha;		// Scalar variable controlling the dissipated energy flow
		float _betar;		// Scalar variable controlling the energy flow due to the non-conservative part of the nominal DS
		float _betarp;	// Scalar variable correcting the non-conservative part of the nominal DS to ensure passivity
		float _betan;   // Scalar variable controlling the energy flow due to the modulation term along the normal direction to the surface
		float _betanp;  // Scalar variable correcting the modulation term along the normal direction to the surface to ensure passivity
		float _pr;			// Power due to the non-conservative part of the nominal DS
		float _pn;			// Power due to the modulation term along the normal direction to the surface
		float _pd;			// Dissipated power
		float _dW;			// Robot's power flow

    //////////////
    // Booleans //
    //////////////
		bool _firstRobotPose;													// Monitor the first robot pose update
		bool _firstRobotTwist;												// Monitor the first robot twist update
		bool _firstWrenchReceived;										// Monitor first force/torque data update
    bool _firstOptitrackPose[NB_TRACKED_OBJECTS];	// Monitor first optitrack markers update
		bool _firstDampingMatrix;											// Monitor first damping matrix update
		bool _optitrackOK;														// Check if all markers position is received
		bool _wrenchBiasOK;														// Check if computation of force/torque sensor bias is OK
		bool _stop;																		// Check for CTRL+C
		bool _adaptMotionModulation;							// Define if we adapt the tangential modulation term to the surface online
		bool _adaptNormalModulation;									// Define if we adapt the normal modualtion term to the surface online
		bool _useOptitrack;									// Define if we adapt the normal modualtion term to the surface online

		/////////////////////////
    // Optitrack variables //
    /////////////////////////
    Eigen::Matrix<float,3,NB_TRACKED_OBJECTS> _markersPosition;       // Markers position in optitrack frame
    Eigen::Matrix<float,3,NB_TRACKED_OBJECTS> _markersPosition0;			// Initial markers position in opittrack frame
    Eigen::Matrix<uint32_t,NB_TRACKED_OBJECTS,1> _markersSequenceID;	// Markers sequence ID
    Eigen::Matrix<uint16_t,NB_TRACKED_OBJECTS,1> _markersTracked;		  // Markers tracked state
		Eigen::Vector3f _p1;																						  // First marker position in the robot frame
		Eigen::Vector3f _p2;																						  // Second marker position in the robot frame
		Eigen::Vector3f _p3;																						  // Third marker position in the robot frame
		uint32_t _optitrackCount;																				  // Counter used to pre-process the optitrack data
		
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
  	float _Fds;
		std::string _fileName;  			// Filename used to log data
		std::ifstream _inputFile;  		// Input stream object to read a text file
		std::ofstream _outputFile;  	// Output stream object to right to a text file
		std::ofstream _outputFile2;  	// Output stream object to right to a text file
		std::mutex _mutex;  					// Mutex variable
		static SurfacePolishing* me;  // Pointer on the instance of the class



		// Dynamic reconfigure (server+callback)
		dynamic_reconfigure::Server<ds_based_contact_tasks::surfacePolishing_paramsConfig> _dynRecServer;
		dynamic_reconfigure::Server<ds_based_contact_tasks::surfacePolishing_paramsConfig>::CallbackType _dynRecCallback;


		Rbf_parameter _rbfAdaptation;
		RbfAdaptation _rbfAdaptation2;
		Eigen::Vector3f _sweepingAttractors[3];
		int _attractorID;
		RbfAdaptation _rbfAdaptationMotionT;
		RbfAdaptation _rbfAdaptationMotionTN;
		RbfAdaptation _rbfAdaptationMotionN;

	public:

		// Class constructor
		SurfacePolishing(ros::NodeHandle &n, double frequency, std::string fileName, 
			               SurfaceType surfaceType, float targetVelocity, 
			               float targetForce,
			               bool adaptNormalModulation);

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

		// Generate circular motion dynamics
		Eigen::Vector3f polishingDS(Eigen::Vector3f position, Eigen::Vector3f attractor);

		// Compute nominal DS
		void computeNominalDS();

		void computeNominalDS2();

		// Compute desired contact force profile
		void computeDesiredContactForceProfile();
		
		void computeDesiredContactForceProfile2();


		// Compute modulation terms along tangential and normal direction to the surface
		void computeModulationTerms();

		// Update scalar variables controlling the tank dynamics
		void updateTankScalars();

		// Compute modulated DS (desired velocity)
		void computeModulatedDS();

		// Compute desired orientation
		void computeDesiredOrientation();
    
  	// Log data to text file
    void logData();

    // Publish data to topics
    void publishData();
    
    // Compute inital markers positon
    void optitrackInitialization();

    // Callback to update the robot pose
    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg);

    // Callback to update the robot twist
    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg);

    // Callback to update the robot wrench (force/torque sensor data)
    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    // Callback to update markers pose from Optitrack
		void updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k); 

		// Check if the marker is tracked
		uint16_t checkTrackedMarker(float a, float b);

		// Callback to update damping matrix form the DS-impedance controller
    void updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg); 

    // Callback for dynamic reconfigure
    void dynamicReconfigureCallback(ds_based_contact_tasks::surfacePolishing_paramsConfig &config, uint32_t level);
};


#endif
