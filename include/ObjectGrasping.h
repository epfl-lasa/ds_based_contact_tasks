#ifndef __OBJECT_GRASPING_H__
#define __OBJECT_GRASPING_H__

#include <signal.h>
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
#include "ros/ros.h"
#include <ros/package.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <dynamic_reconfigure/server.h>
#include "ds_based_contact_tasks/objectGrasping_paramsConfig.h"
#include "Eigen/Eigen"
#include "sg_filter.h"
#include "Workspace.h"

#define NB_FT_SENSOR_SAMPLES 50      // Number of force torque sensors' samples used for initial calibration (compute the offsets)
#define NB_OPTITRACK_SAMPLES 100     // Number of optitrack samples used for initial objects' pose estimation
#define NB_ROBOTS 2                  // Number of robots
#define NB_TRACKED_OBJECTS 6         // Number of objects tracked by the motion capture system (optitrack)
#define MOVING_FORCE_WINDOW_SIZE 10  // Window's size used to average the force data and detect peristent contact

class ObjectGrasping 
{
	public:
    // Exection mode:
    // REACHING_GRASPING_ONLY: The two robots reach and grasp the object
    // REACHING_GRASPING_MANIPULATING: The two robots reach, grasp and move the object to a predefined position                               
    enum Mode {REACHING_GRASPING = 0, REACHING_GRASPING_MANIPULATING = 1};

    // Contact state:
    // CONTACT: Both robots are in contact with the object
    // CLOSE_TO_CONTACT: Both robots are close to make contact with the object
    // NO_CONTACT: Both robots are not in contact with the object
    enum ContactState {CONTACT = 0, CLOSE_TO_CONTACT = 1, NO_CONTACT = 2};

    // Robot ID: left or right
	  enum ROBOT {LEFT = 0, RIGHT = 1};

    // Optitrack object ID:
    // ROBOT_BASIS_{LEFT/RIGHT} Markers placed at the basis of the robot
    // P{i=1,..4} Markers placed on the target object
  	enum ObjectID {ROBOT_BASIS_LEFT = 0, ROBOT_BASIS_RIGHT = 1, P1 = 2, P2 = 3, P3 = 4, P4 = 5};

	private:
    ///////////////////
  	// ROS variables //
    ///////////////////
  	ros::NodeHandle _nh;  // Ros node handle
  	ros::Rate _loopRate;  // Ros loop rate [Hz]
  	float _dt;            // Loop timestep [s]

    //////////////////////////////
    // Subscribers declarations //
    //////////////////////////////
    ros::Subscriber _subRobotPose[NB_ROBOTS];             // Subscribe to robots' pose
    ros::Subscriber _subRobotTwist[NB_ROBOTS];            // Subscribe to robots' twist
    ros::Subscriber _subForceTorqueSensor[NB_ROBOTS];     // Subscribe to force torque sensors
    ros::Subscriber _subOptitrackPose[NB_TRACKED_OBJECTS];  // Subscribe to optitrack markers' pose
    ros::Subscriber _subDampingMatrix[NB_ROBOTS];         // Subscribe to damping matrices of the DS-impedance controller

    ///////////////////////////
    // Publisher declaration //
    ///////////////////////////
    ros::Publisher _pubDesiredTwist[NB_ROBOTS];        // Publish desired twist to DS-impdedance controller
    ros::Publisher _pubDesiredOrientation[NB_ROBOTS];  // Publish desired orientation to DS-impedance controller
    ros::Publisher _pubFilteredWrench[NB_ROBOTS];      // Publish filtered measured wrench
    ros::Publisher _pubNormalForce[NB_ROBOTS];         // Publish measured normal force to the surface
    ros::Publisher _pubMarker;                         // Publish marker (RVIZ) 
    ros::Publisher _pubTaskAttractor;                  // Publish attractor on surface (RVIZ)
    
    //////////////////////////////////////////////////
    // Subsciber and publisher messages declaration //
    //////////////////////////////////////////////////
    geometry_msgs::Pose _msgRealPose;
    geometry_msgs::Pose _msgDesiredPose;
    geometry_msgs::Quaternion _msgDesiredOrientation;
    geometry_msgs::Twist _msgDesiredTwist;
    visualization_msgs::Marker _msgObjectMarker;
    visualization_msgs::Marker _msgArrowMarker;
    geometry_msgs::PointStamped _msgTaskAttractor;
    geometry_msgs::WrenchStamped _msgFilteredWrench;
    
    ///////////////////////////
    // Tool characteristics ///
    ///////////////////////////
    float _toolMass;                             // Tool mass [kg]
    float _toolOffsetFromEE;                     // Tool offset along z axis of end effector [m]             
    Eigen::Vector3f _toolComPositionFromSensor;  // Offset of the tool [m] (3x1)
    Eigen::Vector3f _gravity;                    // Gravity vector [m/s^2] (3x1)
    Eigen::Vector3f _objectDim;                  // Object dimensions [m] (3x1)
   
    ////////////////////////////
    // Tools' state variables //
    ////////////////////////////
    Eigen::Vector3f _x[NB_ROBOTS];                        // Position [m] (3x1)
    Eigen::Vector4f _q[NB_ROBOTS];                        // Current quaternion (4x1)
    Eigen::Vector4f _qinit[NB_ROBOTS];                    // Initial quaternion (4x1)
    Eigen::Matrix3f _wRb[NB_ROBOTS];                      // Orientation matrix (3x3)
    Eigen::Vector3f _v[NB_ROBOTS];                        // Velocity [m/s] (3x1)
    Eigen::Vector3f _w[NB_ROBOTS];                        // Angular velocity [rad/s] (3x1)
    Eigen::Matrix<float,6,1> _wrench[NB_ROBOTS];          // Wrench [N and Nm] (6x1)
    Eigen::Matrix<float,6,1> _wrenchBias[NB_ROBOTS];      // Wrench bias [N and Nm] (6x1)
    Eigen::Matrix<float,6,1> _filteredWrench[NB_ROBOTS];  // Filtered wrench [N and Nm] (6x1)
    float _normalForce[NB_ROBOTS];                        // Normal force to the surface [N] 
    int _wrenchCount[NB_ROBOTS];                          // Counter used to pre-process the force data
    Eigen::Matrix3f _D[NB_ROBOTS];                        // Damping matrix used by the DS-impedance controller (3x3)
    float _d1[NB_ROBOTS];                                 // Damping gain along the desired velocity direction (DS-impedance controller)
    ContactState _contactState;                           // Contact state with the object
    float _c;                                             // Contact value (1 = CONTACT, 0 otherwise)

    ////////////////////////////
    // Tool control variables //
    ////////////////////////////
    Eigen::Vector3f _xd[NB_ROBOTS];      // Desired robots' position [m] (3x1)
    Eigen::Vector4f _qd[NB_ROBOTS];      // Desired robots' quaternion (4x1)
    Eigen::Vector4f _qdPrev[NB_ROBOTS];  // Desired robots' previous quaternion (4x1)
    Eigen::Vector3f _omegad[NB_ROBOTS];  // Desired robots' angular velocity [rad/s] (3x1)
    Eigen::Vector3f _fx[NB_ROBOTS];      // Nominal DS [m/s] (3x1)
    Eigen::Vector3f _fxc[NB_ROBOTS];     // Desired conservative parts of the nominal DS [m/s] (3x1)
    Eigen::Vector3f _fxr[NB_ROBOTS];     // Desired non-conservative parts of the nominal DS [m/s] (3x1)
    Eigen::Vector3f _fxn[NB_ROBOTS];     // Modulation velocity terms along normal direction [m/s] (3x1)
    Eigen::Vector3f _fxp[NB_ROBOTS];     // Corrected nominal DS to ensure passivity [m/s] (3x1)
    Eigen::Vector3f _fxnp[NB_ROBOTS];    // Corrected modulation terms along normal direction to ensure passivity [m/s] (3x1)
    Eigen::Vector3f _vd[NB_ROBOTS];      // Desired modulated DS [m/s] (3x1)
    float _targetForce;                  // Target force in contact [N]
    float _Fd[NB_ROBOTS];                // Desired force profiles [N]
    float _Fdp[NB_ROBOTS];               // Corrected desired force profiles to ensure passivity [N]

    ////////////////////
    // Task variables //
    ////////////////////
    Mode _mode;                                       // Execution mode
    Workspace _workspace;                             // Instance of the Workspace class used to measure the reachibility of a specific position in the robots' workspace
    Eigen::Vector3f _objectAttractor;                 // Attractor position for the object [m] (3x1)
    Eigen::Vector3f _n[NB_ROBOTS];                    // Normal vector to surface object for each robot (3x1)
    Eigen::Vector3f _xC;                              // Center position between the two robots [m] (3x1)
    Eigen::Vector3f _xD;                              // Distance vector between the two robots [m] (3x1)
    Eigen::Vector3f _xoC;                             // Measured object center position [m] (3x1)
    Eigen::Vector3f _xoD;                             // Measured object dimension vector [m] (3x1)
    Eigen::Vector3f _xhC;                             // Home robots' center positon [m] (3x1)
    Eigen::Vector3f _xhD;                             // Home robots' distance vector [m] (3x1)
    Eigen::Vector3f _xdC;                             // Desired center position [m] (3x1)
    Eigen::Vector3f _xdD;                             // Desired distance vector [m] (3x1)
    Eigen::Vector3f _vdC;                             // Desired center position dynamics [m/s] (3x1)
    Eigen::Vector3f _vdD;                             // Desired distance vector dynamics [m/s] (3x1)
    float _eD;                                        // Error to desired distance vector [m]                       
    float _eoD;                                       // Error to object dimension vector [m]                       
    float _eC;                                        // Error to desired center position [m]
    float _eoC;                                       // Error to object center position [m]  
    std::deque<float> _normalForceWindow[NB_ROBOTS];  // Moving window saving the robots' measured normal force to the object's surface [N]     
    float _normalForceAverage[NB_ROBOTS];             // Average normal force measured through the force windows [N]
 
    /////////////////////////////////////////////
    // Normal modulation adaptation parameters //
    /////////////////////////////////////////////
    float _deltaF[NB_ROBOTS];               // Force correction offset [N]
    float _epsilonF;                        // Adaptation rate
    float _epsilonF0;                       // Forgetting rate (to kill the correction term when contact is lost)
    float _gammaF;                          // Fraction of the desired force profile used to bound the correction term

    ///////////////////////
    // Tanks' parameters //
    ///////////////////////
    float _s[NB_ROBOTS];       // Current tanks' level
    float _smax;               // Max tank level
    float _alpha[NB_ROBOTS];   // Scalar variable controlling the dissipated energy flow
    float _betar[NB_ROBOTS];   // Scalar variable controlling the energy flow due to the non-conservative parts of the nominal DS
    float _betarp[NB_ROBOTS];  // Scalar variable correcting the non-conservative parts of the nominal DS to ensure passivity
    float _betan[NB_ROBOTS];   // Scalar variable controlling the energy flow due to the modulation terms along the normal direction to the surface
    float _betanp[NB_ROBOTS];  // Scalar variable correcting the modulation terms along the normal direction to the surface to ensure passivity
    float _pr[NB_ROBOTS];      // Power due to the non-conservative parts of the nominal DS
    float _pn[NB_ROBOTS];      // Power due to the modulation terms along the normal direction to the surface
    float _pd[NB_ROBOTS];      // Dissipated power
    float _dW[NB_ROBOTS];      // Robots' power flow

    //////////////
    // Booleans //
    //////////////
    bool _firstRobotPose[NB_ROBOTS];               // Monitor the first robots' pose update
    bool _firstRobotTwist[NB_ROBOTS];              // Monitor the first robots' twist update
    bool _firstWrenchReceived[NB_ROBOTS];          // Monitor first force/torque data update
    bool _firstOptitrackPose[NB_TRACKED_OBJECTS];  // Monitor first optitrack markers update
    bool _firstDampingMatrix[NB_ROBOTS];           // Monitor first damping matrices' update
    bool _firstObjectPose;                         // Monitor first object pose update
    bool _optitrackOK;                             // Check if all markers position is received
    bool _wrenchBiasOK[NB_ROBOTS];                 // Check if computation of force/torque sensor bias is OK
    bool _stop;                                    // Check for CTRL+C
    bool _objectGrasped;                           // Check if the object is grasped
    bool _objectReachable;                         // Check if object is reachable by both robots
    bool _goHome;                                  // check for goHome state (object not reachable+ not grasped)
    bool _useForceSensor;                          // check for goHome state (object not reachable+ not grasped)
    bool _adaptNormalModulation;                   // Adapt normal modulation based on the force error
    
    /////////////////////////
    // Optitrack variables //
    /////////////////////////
    Eigen::Matrix<float,3,NB_TRACKED_OBJECTS> _markersPosition;       // Markers position in optitrack frame
    Eigen::Matrix<float,3,NB_TRACKED_OBJECTS> _markersPosition0;      // Initial markers position in opittrack frame
    Eigen::Matrix<uint32_t,NB_TRACKED_OBJECTS,1> _markersSequenceID;  // Markers sequence ID
    Eigen::Matrix<uint16_t,NB_TRACKED_OBJECTS,1> _markersTracked;     // Markers tracked state
    Eigen::Vector3f _p1;                                              // First marker position in the right robot frame
    Eigen::Vector3f _p2;                                              // Second marker position in the right robot frame
    Eigen::Vector3f _p3;                                              // Third marker position in the right robot frame
    Eigen::Vector3f _p4;                                              // Fourth marker position in the right robot frame
    Eigen::Vector3f _leftRobotOrigin;                                 // Left robot basis position in the right robot frame
    uint32_t _optitrackCount;                                         // Counter used to pre-process the optitrack data

    ////////////////////
    // User variables //
    ////////////////////
    float _velocityLimit;           // Velocity limit [m/s]
    float _filteredForceGain;       // Filtering gain for force/torque sensor
    Eigen::Vector3f _offset;        // Object attractor offset on surface [m] (3x1)
    float _graspingForceThreshold;  // Grasping force threshold [N]

    /////////////////////
    // Other variables //
    /////////////////////
    SGF::SavitzkyGolayFilter _xCFilter;    // Filter used for the object's center position
    SGF::SavitzkyGolayFilter _xDFilter;    // Filter used for the object's dimension vector
    SGF::SavitzkyGolayFilter _zDirFilter;  // Filter used for the object' z vector
    std::mutex _mutex;                     // Mutex variable
    std::string _filename;                 // Filename used to log data
    std::ifstream _inputFile;              // Input stream object to read a text file
    std::ofstream _outputFile;             // Output stream object to right to a text file

    static ObjectGrasping* me; // Pointer on the instance of the class

    // Dynamic reconfigure (server+callback)
    dynamic_reconfigure::Server<ds_based_contact_tasks::objectGrasping_paramsConfig> _dynRecServer;
    dynamic_reconfigure::Server<ds_based_contact_tasks::objectGrasping_paramsConfig>::CallbackType _dynRecCallback;

  public:

    // Class constructor
		ObjectGrasping(ros::NodeHandle &n, double frequency, std::string filename, Mode mode, float targetForce, bool adaptNormalModulation);

    // initialize node
		bool init();

    // Run node
		void run();

	private:

    // Callback called when CTRL is detected to stop the node		
		static void stopNode(int sig);

    // Compute command to be sent to the DS-impedance controller
    void computeCommand();

    // Compute object pose (position+orientation)
    void computeObjectPose();

    // Check if object is reachable
    void isObjectReachable();

    // Update contact state with the surface
    void updateContactState();
                
    // Compute desired contact force profile
    void computeDesiredContactForceProfile();

    // Compute modulation terms along normal direction to the surface
    void computeModulationTerms();
    
    // Compute nominal DS
    void computeNominalDS();

    // Update scalar variables controlling the tank dynamics
    void updateTankScalars();

    // Compute modulated DS
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
    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k);

    // Callback to update the robot twist
    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k);

    // Callback to update the robot wrench (force/torque sensor data)
    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);

    // Callback to update markers pose from Optitrack
    void updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k); 

    // Check if the marker is tracked
    uint16_t checkTrackedMarker(float a, float b);

    // Callback to update damping matrix from the DS-impedance controller
    void updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int k); 

    // Callback for dynamic reconfigure
    void dynamicReconfigureCallback(ds_based_contact_tasks::objectGrasping_paramsConfig &config, uint32_t level);
};


#endif
