#include "SurfaceLearning.h"
#include "Utils.h"

SurfaceLearning* SurfaceLearning::me = NULL;

SurfaceLearning::SurfaceLearning(ros::NodeHandle &n, double frequency, std::string fileName, 
                                 Mode mode, float C, float sigma, float epsilonTube, 
                                 bool generateDataset, bool addDataOnsurface):
  _nh(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _fileName(fileName),
  _mode(mode),
  _C(C),
  _sigma(sigma),
  _epsilonTube(epsilonTube),
  _generateDataset(generateDataset),
  _addDataOnSurface(addDataOnsurface)
{
  me = this;

  _gravity << 0.0f, 0.0f, -9.80665f;
  // _toolComPositionFromSensor << 0.0f,0.0f,0.02f;
  // _toolOffsetFromEE = 0.15f;
  // _toolMass = 0.1f;
  _toolComPositionFromSensor << 0.0f,0.0f,0.02f;
  _toolOffsetFromEE = 0.155f;
  _toolMass = 0.08f;

  _x.setConstant(0.0f);
  _q.setConstant(0.0f);
  _wrenchBiasOK = false;
  _wrenchCount = 0;
  _wrenchBias.setConstant(0.0f);
  _wrench.setConstant(0.0f);
  _filteredWrench.setConstant(0.0f);
  _normalDistance = 0.0f;

  _xd.setConstant(0.0f);
  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd.setConstant(0.0f);

  _n << 0.0f, 0.0f, 1.0f;
  _datapointID = 0;

  _forceThreshold = 3.0f;
  _heightThreshold = 0.1f;
  _heightOffset = 0.4f;

  _firstRobotPose = false;
  _firstRobotTwist = false;
  _firstWrenchReceived = false;
  for(int k = 0; k < NB_TRACKED_OBJECT; k++)
  {
    _firstOptitrackPose[k] = false;
  }
  _wrenchBiasOK = false;
  _stop = false;
  _optitrackOK = false;

  _markersPosition.setConstant(0.0f);
  _markersPosition0.setConstant(0.0f);
  _markersSequenceID.setConstant(0);
  _markersTracked.setConstant(0);
  _optitrackCount = 0;

  _filteredForceGain = 0.9f;

  _msgArrowMarker.header.frame_id = "world";
  _msgArrowMarker.header.stamp = ros::Time();
  _msgArrowMarker.ns = "marker_test_arrow";
  _msgArrowMarker.id = 0;
  _msgArrowMarker.type = visualization_msgs::Marker::ARROW;
  _msgArrowMarker.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point p1, p2;
  p1.x = 0.0f;
  p1.y = 0.0f;
  p1.z = 0.0f;
  p2.x = 0.0f+0.3f*_n(0);
  p2.x = 0.0f+0.3f*_n(1);
  p2.x = 0.0f+0.3f*_n(2);
  _msgArrowMarker.scale.x = 0.05;
  _msgArrowMarker.scale.y = 0.1;
  _msgArrowMarker.scale.z = 0.1;
  _msgArrowMarker.color.a = 1.0;
  _msgArrowMarker.color.r = 1.0;
  _msgArrowMarker.color.g = 0.0;
  _msgArrowMarker.color.b = 0.0;
  _msgArrowMarker.points.push_back(p1);
  _msgArrowMarker.points.push_back(p2);
}


bool SurfaceLearning::init() 
{
  // Subscriber definitions
  _subRobotPose = _nh.subscribe("/lwr/ee_pose", 1, &SurfaceLearning::updateRobotPose, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist = _nh.subscribe("/lwr/ee_vel", 1, &SurfaceLearning::updateRobotTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor = _nh.subscribe("/ft_sensor/netft_data", 1, &SurfaceLearning::updateRobotWrench, this, ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[ROBOT_BASIS] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/robot/pose", 1, boost::bind(&SurfaceLearning::updateOptitrackPose,this,_1,ROBOT_BASIS),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P1] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/p1/pose", 1, boost::bind(&SurfaceLearning::updateOptitrackPose,this,_1,P1),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P2] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/p2/pose", 1, boost::bind(&SurfaceLearning::updateOptitrackPose,this,_1,P2),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P3] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/p3/pose", 1, boost::bind(&SurfaceLearning::updateOptitrackPose,this,_1,P3),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  
  // Publisher definitions
  _pubDesiredTwist = _nh.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation = _nh.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubFilteredWrench = _nh.advertise<geometry_msgs::WrenchStamped>("SurfaceLearning/filteredWrench", 1);
  _pubMarker = _nh.advertise<visualization_msgs::Marker>("SurfaceLearning/markers", 10);

  signal(SIGINT,SurfaceLearning::stopNode);

  ROS_INFO("[SurfaceLearning]: Filename: %s", _fileName.c_str());

  if(_mode == COLLECTING_DATA)
  {
    ROS_INFO("[SurfaceLearning]: Mode: COLLECTING_DATA");
  }
  else if(_mode == LEARNING)
  {
    ROS_INFO("[SurfaceLearning]: Mode: LEARNING");
    ROS_INFO("[SurfaceLearning]: C: %f", _C);
    ROS_INFO("[SurfaceLearning]: Sigma: %f", _sigma);
    ROS_INFO("[SurfaceLearning]: Epsilon tube: %f", _epsilonTube);
    ROS_INFO("[SurfaceLearning]: Generate dataset on surface: %d", _generateDataset);
    ROS_INFO("[SurfaceLearning]: Use data on surface: %d", _addDataOnSurface);
  }
  else if(_mode == TESTING)
  {
    ROS_INFO("[SurfaceLearning]: Mode: TESTING");
  }
  else
  {
    ROS_ERROR("[SurfaceLearning]: Mode not recognized");
    return false;
  }


  if(_mode == COLLECTING_DATA)
  {
    _outputFile.open(ros::package::getPath(std::string("ds_based_contact_tasks"))+"/data_surface/"+_fileName+"_raw_data.txt");
    if(!_outputFile.is_open())
    {
      ROS_ERROR("[SurfaceLearning]: Cannot open raw data file, the data_surface folder might be missing");
      return false;
    } 
  }
  else if(_mode == LEARNING)
  {
    _optitrackOK = true; 
  }
  else if(_mode == TESTING)
  {

    std::string modelPath = ros::package::getPath(std::string("ds_based_contact_tasks"))+"/data_surface/"+_fileName+"_svmgrad_model.txt";
    _inputFile.open(modelPath);
    if(!_inputFile.is_open())
    {
      ROS_ERROR("[SurfaceLearning]: Cannot svmgrad model model file");
      return false;
    }
    else
    {
      _inputFile.close();
      _svm.loadModel(modelPath);
    }    
  }

  if (_nh.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("[SurfaceLearning]: The modulated ds node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("[SurfaceLearning]: The ros node has a problem.");
    return false;
  }
}


void SurfaceLearning::run()
{
  while (!_stop) 
  {
    if(_firstRobotPose && _firstRobotTwist && _wrenchBiasOK &&
       _firstOptitrackPose[ROBOT_BASIS] && _firstOptitrackPose[P1] &&
       _firstOptitrackPose[P2] && _firstOptitrackPose[P3])
    {

      _mutex.lock();
      // Wait for marker position initialization from optitrack
      if(!_optitrackOK)
      {
        optitrackInitialization();
      }
      else
      {
        updateSurfaceFrame();

        switch(_mode)
        {
          case COLLECTING_DATA:
          {
            // Collect data
            collectData();  
            break;
          }
          case LEARNING:
          {
            // Generate dataset
            if(_generateDataset)
            {
              generateDataset();
            }

            // Learn surface model from dataset
            learnSurfaceModel();
            _stop = true;

            break;
          }
          case TESTING:
          {
            // Compute surface's pose
            testSurfaceModel();
            break;
          }
          default:
          {
            break;
          }
        }

        // Compute command 
        computeCommand();

        // Publish data to topics
        publishData();

      }
        _mutex.unlock();
    }
    ros::spinOnce();
    _loopRate.sleep();
  }

  // Send zero velocity to the robot
  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd = _q;

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  if(_mode == COLLECTING_DATA)
  {
    _outputFile.close();
  }
  
  ros::shutdown();
}


void SurfaceLearning::stopNode(int sig)
{
  me->_stop = true;
}


void SurfaceLearning::updateSurfaceFrame()
{
  // The three markers are positioned on the surface to form an angle of 90 deg:
  // P1 ----- P2
  // |       
  // |
  // P3
  // Compute markers position in the robot frame
  _p1 = _markersPosition.col(P1)-_markersPosition0.col(ROBOT_BASIS);
  _p2 = _markersPosition.col(P2)-_markersPosition0.col(ROBOT_BASIS);
  _p3 = _markersPosition.col(P3)-_markersPosition0.col(ROBOT_BASIS);
  std::cerr << _p1.transpose() << std::endl;

  // Compute surface frame, wRs is the rotation matrix for the surface frame to the world frame  
  _wRs.col(0) = (_p1-_p3).normalized();
  _wRs.col(1) = (_p1-_p2).normalized();
  _wRs.col(2) = ((_wRs.col(0)).cross(_wRs.col(1))).normalized();
}


void SurfaceLearning::computeCommand()
{
  _vd.setConstant(0.0f);

  // Compute desired orientation
  computeDesiredOrientation();
}


void SurfaceLearning::computeDesiredOrientation()
{
  if(_mode == TESTING) // Align tool orientation with the normal vector to the surface
  {
    // Compute rotation error between current orientation and plane orientation using Rodrigues' law
    Eigen::Vector3f k;
    k = (_wRb.col(2)).cross(_n);
    float c = (_wRb.col(2)).transpose()*(_n);  
    float s = k.norm();
    k /= s;
    
    Eigen::Matrix3f K;
    K << Utils<float>::getSkewSymmetricMatrix(k);

    Eigen::Matrix3f Re;
    if(fabs(s)< FLT_EPSILON)
    {
      Re = Eigen::Matrix3f::Identity();
    }
    else
    {
      Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
    }
    
    // Convert rotation error into axis angle representation
    Eigen::Vector3f omega;
    float angle;
    Eigen::Vector4f qtemp = Utils<float>::rotationMatrixToQuaternion(Re);
    Utils<float>::quaternionToAxisAngle(qtemp,omega,angle);

    // Compute final quaternion on plane
    Eigen::Vector4f qf = Utils<float>::quaternionProduct(qtemp,_q);

    // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the plane
    _qd = Utils<float>::slerpQuaternion(_q,qf,1.0f-std::tanh(5.0f*_normalDistance));
    // _qd = slerpQuaternion(_q,qf,1.0f);

    // Compute needed angular velocity to perform the desired quaternion
    Eigen::Vector4f qcurI, wq;
    qcurI(0) = _q(0);
    qcurI.segment(1,3) = -_q.segment(1,3);
    wq = 5.0f*Utils<float>::quaternionProduct(qcurI,_qd-_q);
    Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
    _omegad = omegaTemp;    
  }
  else if(_mode == COLLECTING_DATA) // The desired orientation is set to the current one such that the user can manipulate the end effector
  {
    _qd = _q;
    _omegad.setConstant(0.0f); 
  }

}


void SurfaceLearning::collectData()
{
  // Compute robot postion in surface frame
  Eigen::Vector3f x;
  x = _wRs.transpose()*(_x-_p1);
    
  // Write data to file
  _outputFile << ros::Time::now() << " "
              << x.transpose() << " "
              << _v.transpose() << " "
              << _filteredWrench.segment(0,3).transpose() << " "
              << _datapointID << std::endl;
}


void SurfaceLearning::generateDataset()
{
  _inputFile.open(ros::package::getPath(std::string("ds_based_contact_tasks"))+"/data_surface/"+_fileName+"_raw_data.txt");
  
  if(!_inputFile.is_open())
  {
    ROS_ERROR("[SurfaceLearning]: Cannot open raw data file");
  }

  std::string line;
  double timeTemp;
  Eigen::Vector3f position,velocity,force;
  uint32_t sequenceID;

  std::vector<Eigen::Vector3f> dataOnSurface;

  // Read raw data file to extract data on the surface from it
  while(std::getline(_inputFile,line))  
  {
    std::stringstream ss;
    ss << line;
    ss >> timeTemp >> position(0) >> position(1) >> position(2) >> 
                      velocity(0) >> velocity(1) >> velocity(2) >>
                      force(0) >> force(1) >> force(2) >> sequenceID;

    // The datapoints extracted are the ones with low height and with enough contact force measured
    if(force.norm()>_forceThreshold && position(2) <= _heightThreshold)
    {
      dataOnSurface.push_back(position);
    }
  }

  ROS_INFO("[SurfaceLearning]: Input data size: %d", (uint32_t) dataOnSurface.size());

  // Copy data on surface on an eigen object for easier manipulation
  Eigen::Matrix<float,Eigen::Dynamic,3, Eigen::RowMajor> Xs;
  Xs.resize(dataOnSurface.size(),3);
  memcpy(Xs.data(),dataOnSurface.data(),dataOnSurface.size()*sizeof(Eigen::Vector3f));

  // Get min/max position on the surface for each dimension
  Eigen::Vector3f XsMin, XsMax;
  for(int k = 0; k< 3; k++)
  {
    XsMin(k) = Xs.col(k).array().minCoeff();
    XsMax(k) = Xs.col(k).array().maxCoeff();
  }

  std::cerr << "[SurfaceLearning]: Min position (for each dimension): " << XsMin.transpose() << std::endl;
  std::cerr << "[SurfaceLearning]: Max position (for each dimension): " << XsMax.transpose() << std::endl;

  // Generate random number between 0 and 1
  srand(time(NULL));
  Eigen::Matrix<float,Eigen::Dynamic,3> R;
  R.resize(DATASET_SIZE,3);
  R.setRandom();
  R.array()+= 1.0f;
  R.array()/=2.0f;

  //////////////////////
  // Generate dataset //
  //////////////////////
  // The first three colum are the x,y,z position, the last one is the normal distance

  // Start with input data by generating points below/above the surface
  // from the random numbers generated
  Eigen::MatrixXf dataset;
  dataset.resize(DATASET_SIZE,4);
  for(int k = 0; k< 3; k++)
  {
    if(k == 2)
    {
      dataset.col(k).array() = XsMin(k)+(XsMax(k)+_heightOffset-XsMin(k))*R.col(k).array();
    }
    else
    {
      dataset.col(k).array() = XsMin(k)+(XsMax(k)-XsMin(k))*R.col(k).array();
    }
  }

  // Compute the output label for each datapoint (= approximated normal distance)
  Eigen::Matrix<float,Eigen::Dynamic,1>  outputIndex;
  outputIndex.resize(dataset.rows());

  Eigen::VectorXf::Index index;
  for(uint32_t k = 0; k < dataset.rows(); k++)
  {
    dataset(k,3) = (dataset.row(k).segment(0,3).replicate(Xs.rows(),1)-Xs).rowwise().norm().array().minCoeff(&index);
    outputIndex(k) = index; 
    if(dataset(k,2)-Xs(index,2)<0.0f)
    {
      dataset(k,3) = -dataset(k,3);
    }
  }

  _inputFile.close();

  // Add data on surface to form the full dataset
  Eigen::MatrixXf datasetFull;
  datasetFull.resize(DATASET_SIZE+Xs.rows(),4);
  datasetFull.setConstant(0.0f);
  datasetFull.block(0,0,DATASET_SIZE,4) = dataset;
  datasetFull.block(DATASET_SIZE,0,Xs.rows(),3) = Xs;

  _outputFile.open(ros::package::getPath(std::string("ds_based_contact_tasks"))+"/data_surface/"+_fileName+"_libsvm_data.txt");

  // Write dataset file to be used by SVM
  if(!_outputFile.is_open())
  {
    ROS_ERROR("[SurfaceLearning]: Cannot open libsvm data file");
  }

  if(_addDataOnSurface)
  {
    for(uint32_t k = 0; k < datasetFull.rows(); k++)
    {
      _outputFile << datasetFull(k,3) << " 1:" << datasetFull(k,0) << " 2:" << datasetFull(k,1) << " 3:" << datasetFull(k,2) << std::endl;
    }
  }
  else
  {
    for(uint32_t k = 0; k < dataset.rows(); k++)
    {
      _outputFile << dataset(k,3) << " 1:" << dataset(k,0) << " 2:" << dataset(k,1) << " 3:" << dataset(k,2) << std::endl;
    }
  }

  _outputFile.close();
}



void SurfaceLearning::learnSurfaceModel()
{
  ROS_INFO("[SurfaceLearning]: Learning surface model ...");
  std::string command;

  // Dataset input file
  std::string dataFile = ros::package::getPath(std::string("ds_based_contact_tasks"))+"/data_surface/"+_fileName+"_libsvm_data.txt";
  
  // Model output file
  std::string modelFile = ros::package::getPath(std::string("ds_based_contact_tasks"))+"/data_surface/"+_fileName+"_libsvm_model.txt";
  float gamma = 1.0f/(2.0f*std::pow(_sigma,2.0f));
  ROS_INFO("[SurfaceLearning]: Gamma value computed from sigma: %f", gamma);

  // Call SVM command
  std::string svmCommand = "svm-train -s 3 -t 2 -c "+std::to_string(_C)+" -g "+std::to_string(gamma)+" -h 0 -p "+std::to_string(_epsilonTube);
  command = svmCommand+" "+dataFile+" "+modelFile;
  ROS_INFO("[SurfaceLearning]: SVM command: %s", command.c_str());
  int val = std::system(command.c_str());
  ROS_INFO("[SurfaceLearning]: libsvm command result: %d", val);

  generateSVMGradModelFile();
}

void SurfaceLearning::generateSVMGradModelFile()
{
  ROS_INFO("[SurfaceLearning]: Generate svm grad model file ...");
  std::string command;
  _inputFile.open(ros::package::getPath(std::string("ds_based_contact_tasks"))+"/data_surface/"+_fileName+"_libsvm_model.txt");
  if(!_inputFile.is_open())
  {
    ROS_ERROR("[SurfaceLearning]: Cannot open libsvm model file");
  }

  std::string modelType, kernelType;
  float gamma, rho;
  int nbClasses, nbSVs;

  std::string line;
  std::getline(_inputFile,line);
  modelType = line.substr(1,' ');
  std::getline(_inputFile,line);
  kernelType = line.substr(1,' ');
  std::getline(_inputFile,line);
  sscanf(line.c_str(),"gamma %f", &gamma);
  std::getline(_inputFile,line);
  sscanf(line.c_str(),"nr_class %d", &nbClasses);
  std::getline(_inputFile,line);
  sscanf(line.c_str(),"total_sv %d", &nbSVs);
  std::getline(_inputFile,line);
  sscanf(line.c_str(),"rho %f",&rho);
  std::getline(_inputFile,line);


  Eigen::Matrix<float,Eigen::Dynamic,3> sv;
  Eigen::VectorXf alpha;

  if(line =="SV")
  {
    sv.resize(nbSVs,3);
    alpha.resize(nbSVs);
    
    for(int k = 0; k < nbSVs; k++)
    {
      std::getline(_inputFile,line);
      sscanf(line.c_str(),"%f 1:%f 2:%f 3:%f", &alpha(k),&sv(k,0),&sv(k,1),&sv(k,2));
    }
  }
  ROS_INFO("[SurfaceLearning]: svm_type %s", modelType.c_str());
  ROS_INFO("[SurfaceLearning]: kernel_type %s", kernelType.c_str());
  ROS_INFO("[SurfaceLearning]: gamma %f", gamma);
  ROS_INFO("[SurfaceLearning]: nr_class %d", nbClasses);
  ROS_INFO("[SurfaceLearning]: total_sv %d", nbSVs);
  ROS_INFO("[SurfaceLearning]: rho %f",rho);

  _inputFile.close();

  _outputFile.open(ros::package::getPath(std::string("ds_based_contact_tasks"))+"/data_surface/"+_fileName+"_svmgrad_model.txt");
  if(!_outputFile.is_open())
  {
    ROS_ERROR("[SurfaceLearning]: Cannot open svnm grad model file");
  }
  else
  {
    _outputFile << sv.cols() << std::endl
                << nbSVs << std::endl
                << -rho << std::endl
                << 1.0f/std::sqrt(2*gamma) << std::endl << std::endl
                << alpha.transpose() << std::endl << std::endl
                << sv.col(0).transpose() << std::endl
                << sv.col(1).transpose() << std::endl
                << sv.col(2).transpose() << std::endl;

    _outputFile.close();
  }
}


void SurfaceLearning::testSurfaceModel()
{
  // Compute robot postion in surface frame
  Eigen::Vector3f x;
  x = _wRs.transpose()*(_x-_p1);
  
  _svm.preComputeKernel(true);
  // We compute the normal distance by evlauating the SVM model
  _normalDistance = _svm.calculateGamma(x.cast<double>());

    // We get the normal vector by evaluating the gradient of the model
  _n = -_svm.calculateGammaDerivative(x.cast<double>()).cast<float>();
  _n = _wRs*_n;
  _n.normalize();
  std::cerr << " [SurfaceLearning]: Normal distance: " << _normalDistance << " Normal vector: " << _n.transpose() << std::endl;
}


void SurfaceLearning::publishData()
{
  // Publish desired twist (passive ds controller)
  _msgDesiredTwist.linear.x  = _vd(0);
  _msgDesiredTwist.linear.y  = _vd(1);
  _msgDesiredTwist.linear.z  = _vd(2);
  _msgDesiredTwist.angular.x = _omegad(0);
  _msgDesiredTwist.angular.y = _omegad(1);
  _msgDesiredTwist.angular.z = _omegad(2);
  _pubDesiredTwist.publish(_msgDesiredTwist);

  // Publish desired orientation
  _msgDesiredOrientation.w = _qd(0);
  _msgDesiredOrientation.x = _qd(1);
  _msgDesiredOrientation.y = _qd(2);
  _msgDesiredOrientation.z = _qd(3);
  _pubDesiredOrientation.publish(_msgDesiredOrientation);

  // Publis measured filtered wrench (force torque sensor data)
  _msgFilteredWrench.header.frame_id = "world";
  _msgFilteredWrench.header.stamp = ros::Time::now();
  _msgFilteredWrench.wrench.force.x = _filteredWrench(0);
  _msgFilteredWrench.wrench.force.y = _filteredWrench(1);
  _msgFilteredWrench.wrench.force.z = _filteredWrench(2);
  _msgFilteredWrench.wrench.torque.x = _filteredWrench(3);
  _msgFilteredWrench.wrench.torque.y = _filteredWrench(4);
  _msgFilteredWrench.wrench.torque.z = _filteredWrench(5);
  _pubFilteredWrench.publish(_msgFilteredWrench);

  // Publish markers for RVIZ vizualisation
  _msgArrowMarker.points.clear();
  geometry_msgs::Point p1, p2;
  p1.x = _x(0);
  p1.y = _x(1);
  p1.z = _x(2);
  p2.x = _x(0)+0.3f*_n(0);
  p2.y = _x(1)+0.3f*_n(1);
  p2.z = _x(2)+0.3f*_n(2);
  _msgArrowMarker.points.push_back(p1);
  _msgArrowMarker.points.push_back(p2);
  _pubMarker.publish(_msgArrowMarker);
}


void SurfaceLearning::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  _msgRealPose = *msg;

  Eigen::Vector3f temp = _x;

  // Update end effecotr pose (position+orientation)
  _x << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
  _q << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
  _wRb = Utils<float>::quaternionToRotationMatrix(_q);
  _x = _x+_toolOffsetFromEE*_wRb.col(2);

  // Make sure to add new points to dataset
  if((_x-temp).norm()>FLT_EPSILON)
  {
    _datapointID++;
    if(_mode == COLLECTING_DATA && _wrenchBiasOK)
    {
      std::cerr << "[SurfaceLearning]: " << "datapoint ID: " <<_datapointID << std::endl;
    }
  }

  if(!_firstRobotPose)
  {
    _firstRobotPose = true;
    _xd = _x;
    _qd = _q;
    _vd.setConstant(0.0f);
  }
}


void SurfaceLearning::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  _v(0) = msg->linear.x;
  _v(1) = msg->linear.y;
  _v(2) = msg->linear.z;
  _w(0) = msg->angular.x;
  _w(1) = msg->angular.y;
  _w(2) = msg->angular.z;

  if(!_firstRobotTwist)
  {
    _firstRobotTwist = true;
  }
}
 
void SurfaceLearning::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

  if(!_wrenchBiasOK && _firstRobotPose)
  {
    Eigen::Vector3f loadForce = _wRb.transpose()*_toolMass*_gravity;
    _wrenchBias.segment(0,3) -= loadForce;
    _wrenchBias.segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _wrenchBias += raw; 
    _wrenchCount++;
    if(_wrenchCount==NB_FT_SAMPLES)
    {
      _wrenchBias /= NB_FT_SAMPLES;
      _wrenchBiasOK = true;
    }
  }

  if(_wrenchBiasOK && _firstRobotPose)
  {
    _wrench = raw-_wrenchBias;
    Eigen::Vector3f loadForce = _wRb.transpose()*_toolMass*_gravity;
    _wrench.segment(0,3) -= loadForce;
    _wrench.segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _filteredWrench = _filteredForceGain*_filteredWrench+(1.0f-_filteredForceGain)*_wrench;
  }

}

void SurfaceLearning::optitrackInitialization()
{
  if(_optitrackCount< NB_OPTITRACK_SAMPLES)
  {
    if(_markersTracked(ROBOT_BASIS))
    {
      _markersPosition0 = (_optitrackCount*_markersPosition0+_markersPosition)/(_optitrackCount+1);
      _optitrackCount++;
    }
    std::cerr << "[SurfaceLearning]: Optitrack Initialization count: " << _optitrackCount << std::endl;
    if(_optitrackCount == 1)
    {
      ROS_INFO("[SurfaceLearning]: Optitrack Initialization starting ...");
    }
    else if(_optitrackCount == NB_OPTITRACK_SAMPLES)
    {
      ROS_INFO("[SurfaceLearning]: Optitrack Initialization done !");
    }
  }
  else
  {
    _optitrackOK = true;
  }
}


void SurfaceLearning::updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k) 
{
  if(!_firstOptitrackPose[k])
  {
    _firstOptitrackPose[k] = true;
  }

  _markersSequenceID(k) = msg->header.seq;
  _markersTracked(k) = checkTrackedMarker(_markersPosition.col(k)(0),msg->pose.position.x);
  _markersPosition.col(k) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(k == (int) ROBOT_BASIS)
  {
    _markersPosition.col(k)(2) -= 0.03f;
  }
}


uint16_t SurfaceLearning::checkTrackedMarker(float a, float b)
{
  if(fabs(a-b)< FLT_EPSILON)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

