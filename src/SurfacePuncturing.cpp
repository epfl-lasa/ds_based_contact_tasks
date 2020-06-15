#include "SurfacePuncturing.h"
#include "Utils.h"
#include <numeric>

SurfacePuncturing* SurfacePuncturing::me = NULL;

SurfacePuncturing::SurfacePuncturing(ros::NodeHandle &n, double frequency, std::string fileName):
  _nh(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _fileName(fileName)
{
  me = this;

  _sim = false;

  _gravity << 0.0f, 0.0f, -9.80665f;
 
  // Version small f/t sensor
  _toolComPositionFromSensor<< 0.0f,0.0f,0.035f;
  _toolOffsetFromEE << 0.0f, -0.028, 0.132f;
  _toolOffsetFromEE << 0.0f, 0.0f, 0.128f+0.015f;
  // _toolOffsetFromEE= 0.23f;
  _toolMass= 0.1f;


  _x.setConstant(0.0f);
  _q.setConstant(0.0f);
  _wrenchBiasOK = false;
  _wrenchBias.setConstant(0.0f);
  _wrench.setConstant(0.0f);
  _filteredWrench.setConstant(0.0f);
  _normalDistance = 0.0f;
  _normalForce = 0.0f;
  _wrenchCount = 0;
  _d1 = 0.0f;
  _D.setConstant(0.0f);

  _xd.setConstant(0.0f);
  _fxc.setConstant(0.0f);
  _fxr.setConstant(0.0f);
  _fx.setConstant(0.0f);
  _fxn.setConstant(0.0f);
  _fxp.setConstant(0.0f);
  _fxnp.setConstant(0.0f);
  _vd.setConstant(0.0f);
  _Fd = 0.0f;
  _omegad.setConstant(0.0f);
  _qd.setConstant(0.0f);

  _n << 0.0f, 0.0f, -1.0f;

  _planeNormal << 0.0f, 0.0f, 1.0f;
  _normalDistanceTolerance = 0.05f;
  _normalForceTolerance = 0.8f;
  _normalForceAverage = 0.0f;
  _c = 0.0f;
 
  _firstRobotPose = false;
  _firstRobotTwist = false;
  _firstWrenchReceived = false;
  _firstDampingMatrix = false;
  _wrenchBiasOK = false;
  _stop = false;

  _velocityLimit = 0.3f;

  _filteredForceGain = 0.9;

  _xs << -0.5f,-0.23,0.250f-_toolOffsetFromEE(2);
  _xa = _xs;
  _attractorOffset = 0.04f;
  _puncturingVelocityLimit = 0.03f;

  Eigen::VectorXf coeff;
  coeff.resize(9);
  coeff << -0.0013f, 0.0311f, 0.1309f, 0.2645f, 0.3289f, 0.2645f, 0.1309f, 0.0311f, -0.0013f;

  _filter.init(coeff);  

  _dNormalForce = 0.0f;
  _prevDNormalForce = 0.0f;
  _ddNormalForce = 0.0f;
  _prevDDNormalForce = 0.0f;
  _picDetected = false;

  // std::cerr << "AAAAAAAAAAAAAAAAAAAA" << std::endl;
  _gp[0] = new GP_32(1,1);
  _gp[0]->load<serialize::TextArchive>("gp_p_p_p", false);
  std::cout << _gp[0]->kernel_function().h_params().transpose() << std::endl;
  _gp[1] = new GP_32(1,1);
  _gp[1]->load<serialize::TextArchive>("gp_t_p_p", false);
  _gp[2] = new GP_32(1,1);
  _gp[2]->load<serialize::TextArchive>("gp_p_p_t", false);
  _forceModel.setConstant(0.0f);

  // std::cerr << "BBBBBBBBBBBBBBBBBBBBBBB" << std::endl;


  _compensation = 0.0f;
  _useModel = false;
  _useAdaptation = false;
  _adaptationRate = 15.0f;
  _fxn.setConstant(0.0f);

  _idModel = 0;
}


bool SurfacePuncturing::init()
{
  // Subscriber definitions
  _subRobotPose = _nh.subscribe("/lwr/ee_pose", 1, &SurfacePuncturing::updateRobotPose, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist = _nh.subscribe("/lwr/ee_vel", 1, &SurfacePuncturing::updateRobotTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor = _nh.subscribe("/ft_sensor/netft_data", 1, &SurfacePuncturing::updateRobotWrench, this, ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix = _nh.subscribe("/lwr/joint_controllers/passive_ds_damping_matrix", 1, &SurfacePuncturing::updateDampingMatrix,this,ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist = _nh.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation = _nh.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubFilteredWrench = _nh.advertise<geometry_msgs::WrenchStamped>("SurfacePuncturing/filteredWrench", 1);
  _pubNormalForce = _nh.advertise<std_msgs::Float32>("SurfacePuncturing/normalForce", 1);
  _pubDepth = _nh.advertise<std_msgs::Float32>("SurfacePuncturing/depth", 1);


  signal(SIGINT,SurfacePuncturing::stopNode);


  ROS_INFO("[SurfacePuncturing]: Filename: %s", _fileName.c_str());

  _outputFile.open(ros::package::getPath(std::string("ds_based_contact_tasks"))+"/data_cutting/"+_fileName+".txt");

  if(!_nh.getParamCached("/lwr/joint_controllers/ds_param/damping_eigval0",_d1))
  {
    ROS_ERROR("[SurfacePuncturing]: Cannot read first eigen value of passive ds controller");
    return false;
  }

  if(!_outputFile.is_open())
  {
    ROS_ERROR("[SurfacePuncturing]: Cannot open output data file, the data_cutting directory might be missing");
    return false;
  }

  return true;
}


void SurfacePuncturing::run()
{
  _timeInit = ros::Time::now().toSec();

  // while (!_stop && ros::Time::now().toSec()-_timeInit < _duration)
  while (!_stop)
  {
    if(allSubscribersOK())
    {
      // Check for update of the DS-impedance controller gain
      ros::param::getCached("/lwr/joint_controllers/ds_param/damping_eigval0",_d1);

      if(_d1<1.0f)
      {
        _d1 = 100.0f;
      }

      // Compute control command
      computeCommand();

      // Publish data to topics
      publishData();

      // Log data
      logData();
    }

    ros::spinOnce();

    _loopRate.sleep();
  }

  // Send zero velocity command to stop the robot
  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd = _q;
  _n.setConstant(0.0f);

  publishData();
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}


void SurfacePuncturing::stopNode(int sig)
{
  me->_stop = true;
}


bool SurfacePuncturing::allSubscribersOK()
{
  bool status = _firstRobotPose && _firstRobotTwist && (_sim || _wrenchBiasOK);

  if(!status)
  {
    std::cerr << "p: " << (int) _firstRobotPose << " t: " << (int) _firstRobotTwist << " w: " << (int) _wrenchBiasOK << std::endl;
  }
  return status;     
}


void SurfacePuncturing::computeCommand()
{
  updateSurfaceInformation();

  // Update contact state
  updateContactState();

  // Compute nominal DS
  computeNominalDS();

  // Compute desired contact force profile
  computeDesiredContactForceProfile();

  // Compute modulation terms
  computeModulationTerms();

  // Compute modulated DS
  computeModulatedDS();
    
  // Compute desired orientation
  computeDesiredOrientation();
}


void SurfacePuncturing::updateSurfaceInformation()
{
  // Plane normal
  _planeNormal << 0.0f, 0.0f, 1.0f;    

  // Normal vector 
  _n = -_planeNormal;
      
  // Normal distanc
  _normalDistance = _x(2)-_xs(2);

  // Compute normal force
  Eigen::Vector3f F = _filteredWrench.segment(0,3);
  Eigen::Vector3f w = _filteredWrench.segment(3,3);
  _prevNormalForce = _normalForce;
  _normalForce = _n.dot(-_wRb*F);

 
  std::cerr << "[SurfacePuncturing]: Normal distance: " << _normalDistance << " Normal force: " << _normalForce << std::endl;    

}


void SurfacePuncturing::updateContactState()
{
  if(_normalForceWindow.size()<MOVING_FORCE_WINDOW_SIZE)
  {
    _normalForceWindow.push_back(_normalForce);
    _normalForceAverage = 0.0f;
  }
  else
  {
    _normalForceWindow.pop_front();
    _normalForceWindow.push_back(_normalForce);
    _normalForceAverage = 0.0f;
    for(int k = 0; k < MOVING_FORCE_WINDOW_SIZE; k++)
    {
      _normalForceAverage+=_normalForceWindow[k];
    }
    _normalForceAverage /= MOVING_FORCE_WINDOW_SIZE;
  }

  if(_normalForceAverage >= _normalForceTolerance && _normalDistance < _normalDistanceTolerance)
  {
    _contactState = CONTACT;
    _c = 1.0f;
  }
  else if(_normalForceAverage < _normalForceTolerance && _normalDistance < _normalDistanceTolerance)
  {
    _contactState = CLOSE_TO_CONTACT;
    _c = 0.0f;
  }
  else
  {
    _contactState = NO_CONTACT;
    _c = 0.0f;
  }
}


void SurfacePuncturing::computeNominalDS()
{
  
  // Nominal DS
  _fx = 2.0f*(_xs-_x);

  // Attractor
  _xa = _xd;
  _xa(2) = _xs(2)-_attractorOffset;
  _fx = 2.0f*(_xa-_x);
  _fx(2) = -_puncturingVelocityLimit;
  if(_x(2)-_xs(2)<-0.038f)
  {
    me->_stop = true;
  }
  // _fx = Utils<float>::bound(_fx,0.f);
  



  if(_contactState==CONTACT)
  {
    _forceWindow.push_front(_normalForce);
    if(_forceWindow.size()>5)
    {
      _forceWindow.pop_back();
    }

    Eigen::VectorXf fw;
    fw.resize(_forceWindow.size());
    for(int m = 0; m < _forceWindow.size(); m++)
    {
      fw(m) = _forceWindow[m];
    }

    Eigen::VectorXd result[3];
    Eigen::Vector3f sumError;

    for(int k = 0; k < 3; k++)
    {
      result[k] = _gp[k]->mu(tools::make_vector(-_normalDistance));
      _forceModel(k) = result[k](0);

      _gpForce[k].push_front(result[k](0));

      std::cerr << k << " : " << result[k](0) << std::endl;
      if(_gpForce[k].size()>5)
      {
        _gpForce[k].pop_back();
      }

      Eigen::VectorXf temp;
      temp.resize(_gpForce[k].size());
      for(int m = 0; m < temp.size(); m++)
      {
        temp(m) = _gpForce[k][m];
      }

      std::cerr << temp.transpose() << std::endl;
      std::cerr << (temp-temp.array().mean()*Eigen::VectorXf::Ones(temp.size())).transpose() << std::endl;
      sumError(k) = ((temp-temp.array().mean()*Eigen::VectorXf::Ones(temp.size()))-
      (fw-fw.array().mean()*Eigen::VectorXf::Ones(fw.size()))).array().abs().mean();
      
      // sumError(k) = temp.sum();
    }

    Eigen::MatrixXf::Index id;
    sumError.minCoeff(&id);
    _idModel = id;
    std::cerr << "Error: " << sumError(0) << " " << sumError(1) << " " << sumError(2) << std::endl;

    std::cerr << "Predicted model: " << _idModel << std::endl;
  }



    //  // _compensation = result(0);
    //  _compensation = 0.0f;
    //  // std::cerr << "compensation: " << _compensation  << std::endl;
    //  if(_compensation >16)
    //  {
    //   _compensation  = 16.0f;
    //  } 
    //  // _fx+=(_compensation/_d1)*_n; 
    //   // std::cerr << "deltav: " << _deltav << " v:" << _n.dot(_v) << std::endl;
   
    // _prevDNormalForce = _dNormalForce;
    // _dNormalForce = (_normalForce-_prevNormalForce)/_dt;
    // _prevDDNormalForce = _ddNormalForce;
    // _ddNormalForce = _dNormalForce-_prevDNormalForce/_dt;

    // if(_ddNormalForce < 0.0f && _prevDDNormalForce > 0.0f)
    // {
    //   _positivePicDForce.push_back(_dNormalForce);
    //   _positivePicTime.push_back(ros::Time::now().toSec());
    // }
    // else if(_ddNormalForce > 0.0f && _prevDDNormalForce < 0.0f && _dNormalForce < 0.0f)
    // {
    //   _negativePicDForce.push_back(_dNormalForce);
    //   _negativePicTime.push_back(ros::Time::now().toSec()); 
    //   if(!_positivePicDForce.empty())
    //   {
    //     float deltaPic = _negativePicDForce.back()-_positivePicDForce.back();
    //     float deltaTime = _negativePicTime.back()-_positivePicTime.back();
    //     if(deltaPic < -12.0f && deltaTime < 0.1f)
    //     {
    //       _picDetected = true;
    //     }
    //   }
    // }
    
    // std::cerr << "Pic detected: " << (int) _picDetected << std::endl;  
    // if(_picDetected)
    // {
    //   _fx.setConstant(0.0f);
    // }
    // std::cerr << xa(2)-_x(2) << std::endl;
  // }
}


void SurfacePuncturing::computeDesiredContactForceProfile()
{
  if(_contactState==CONTACT)// && _attractorID == 2)
  {
    if(_useModel)
    {

      Eigen::VectorXd temp;
      temp =  _gp[_idModel]->mu(tools::make_vector(-_normalDistance));
      _Fd = temp(0);
    }
    else
    {
      _Fd = 0.0f;
    }
  }
  else if(_contactState==CLOSE_TO_CONTACT)//  && _attractorID == 2)
  {
    _Fd = 0.0f;
  }
  else
  {
    _Fd = 0.0f;
  }

  std::cerr << "[SurfacePuncturing]: Fd: " << _Fd << " c: " << _c << std::endl;
}


void SurfacePuncturing::computeModulationTerms()
{
  if(_useModel)
  {
    // _compensation = _Fd/_d1;
    if(_useAdaptation)
    {
      if(_contactState==CONTACT)
      {
        _compensation += _dt*_adaptationRate*_n.transpose()*(_fx-_v);
        _compensation = Utils<float>::bound(_compensation,0.0f,0.2f);
      }
      else
      {
        _compensation = 0.0f;
      }    
    }
    else
    {
      _compensation = 0.0f;
    }

    _fxn = (_compensation+_Fd/_d1)*_n;
    std::cerr << "Compensation: " << _compensation << std::endl;


  }
  else
  {
    if(_contactState==CONTACT)
    {
      _compensation += _dt*_adaptationRate*_n.transpose()*(_fx-_v);
      _compensation = Utils<float>::bound(_compensation,0.0f,0.2f);
      _fxn = _compensation*_n;      
      std::cerr << "Compensation: " << _compensation << std::endl;
    }   

  }
}


void SurfacePuncturing::computeModulatedDS()
{

  // Compute corrected nominal DS and modulation terms
  _fxp = _fx;
  _fxnp = _fxn;

  // Compute modulated DS
  _vd = _fxp+_fxnp;

  // Bound modulated DS for safety
  _vd = Utils<float>::bound(_vd,0.3f);

  std::cerr << "[SurfacePuncturing]: fx: " << _fx.norm() << std::endl;
  std::cerr << "[SurfacePuncturing]: vd: " << _vd.norm() << " v: " << _v.norm() <<std::endl;
}


void SurfacePuncturing::computeDesiredOrientation()
{
  // Compute rotation error between current orientation and plane orientation using Rodrigues' law

  _qd << 0.0f, 0.0f, 1.0f, 0.0f;
  // Eigen::Matrix3f Rd;
  // // float theta = 20.0f*M_PI/180.0f;
  // float theta = 0.0f*M_PI/180.0f;
  
  // Rd << -1.0f, 0.0f, 0.0f,
  //         0.0f, std::cos(theta), std::sin(theta),
  //         0.0f, std::sin(theta), -std::cos(theta);

  // _qd = Utils<float>::rotationMatrixToQuaternion(Rd);


  if(_q.dot(_qd)<0)
  {
    _qd *=-1.0f;
  }

  Eigen::Vector4f qe, qinv;
  qinv = _q;
  qinv.segment(1,3) *=-1.0f;

  qe = Utils<float>::quaternionProduct(_qd,qinv);
  Eigen::Vector3f axis;
  float angle;
  Utils<float>::quaternionToAxisAngle(qe,axis,angle);
  std::cerr << angle << " " << Utils<float>::bound(angle,-0.2f,0.2f) <<std::endl;
  qe = Utils<float>::axisAngleToQuaterion(axis,Utils<float>::bound(angle,-0.2f,0.2f));

  _qd = Utils<float>::quaternionProduct(qe,_q);



  // _qd = qf;
  // Compute needed angular velocity to perform the desired quaternion
  Eigen::Vector4f qcurI, wq;
  qcurI(0) = _q(0);
  qcurI.segment(1,3) = -_q.segment(1,3);
  wq = 5.0f*Utils<float>::quaternionProduct(qcurI,_qd-_q);
  Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
  _omegad = omegaTemp;
}

void SurfacePuncturing::logData()
{
  Eigen::Vector3f F, Ffiltered;
  F = -_wRb*_wrench.segment(0,3);
  Ffiltered = -_wRb*_filteredWrench.segment(0,3);
  _outputFile << ros::Time::now() << " "
              << _x.transpose() << " "
              << _v.transpose() << " "
              << _n.transpose() << " "
              << _fx.transpose() << " "
              << _fxn.transpose() << " "
              << _vd.transpose() << " "
              << Ffiltered.transpose() << " "
              << F.transpose() << " "
              << _normalDistance << " "
              << _c << " "
              << _d1 << " "
              << _adaptationRate << " "
              << (int) _useModel << " "
              << _idModel << " "
              << _forceModel.transpose() << std::endl;
              // << _prevNormalForce << " " 
              // << _prevDNormalForce << " " 
              // << _dNormalForce << " " 
              // << _prevDDNormalForce << " " 
              // << _ddNormalForce << " " 
              // << (int) _picDetected << " " 
              // << _fxp.transpose() << " "
              // << _fxnp.transpose() << " "
              // << _vd.transpose() << " "
              // << _n.transpose() << " "
              // << _wRb.col(2).transpose() << " "
              // << _normalDistance << " "
              // << _normalForce << " "
              // << _Fd << " "
              // << _c << " " 
              // << _xd.transpose() << " " 
              // << _xc.transpose() << std::endl;
}


void SurfacePuncturing::publishData()
{
  // Publish desired twist (passive ds controller)
  _msgDesiredTwist.linear.x  = _vd(0);
  _msgDesiredTwist.linear.y  = _vd(1);
  _msgDesiredTwist.linear.z  = _vd(2);

  // Convert desired end effector frame angular velocity to world frame
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

  _msgFilteredWrench.header.frame_id = "world";
  _msgFilteredWrench.header.stamp = ros::Time::now();
  _msgFilteredWrench.wrench.force.x = _filteredWrench(0);
  _msgFilteredWrench.wrench.force.y = _filteredWrench(1);
  _msgFilteredWrench.wrench.force.z = _filteredWrench(2);
  _msgFilteredWrench.wrench.torque.x = _filteredWrench(3);
  _msgFilteredWrench.wrench.torque.y = _filteredWrench(4);
  _msgFilteredWrench.wrench.torque.z = _filteredWrench(5);
  _pubFilteredWrench.publish(_msgFilteredWrench);

  // Publish normal force
  std_msgs::Float32 msg;
  msg.data = _normalForce;
  _pubNormalForce.publish(msg);

  msg.data = _normalDistance;
  _pubDepth.publish(msg);
}


void SurfacePuncturing::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  _msgRealPose = *msg;

  Eigen::Vector3f temp = _x;

  // Update end effecotr pose (position+orientation)
  _x << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
  _q << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
  _wRb = Utils<float>::quaternionToRotationMatrix(_q);
  // _x = _x+_toolOffsetFromEE*_wRb.col(2);
  _x = _x+_wRb*_toolOffsetFromEE;

  if(!_firstRobotPose)
  {
    _firstRobotPose = true;
    _xd = _x;
    _qd = _q;
    _vd.setConstant(0.0f);
  }
}


void SurfacePuncturing::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg)
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


void SurfacePuncturing::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;
  // raw(0) = msg->wrench.force.y;
  // raw(1) = -msg->wrench.force.x;
  // raw(2) = msg->wrench.force.z;
  // raw(3) = msg->wrench.torque.y;
  // raw(4) = -msg->wrench.torque.x;
  // raw(5) = msg->wrench.torque.z;

  // std::cerr << raw.transpose() << std::endl;
  Eigen::Matrix3f temp;
  // temp << 0.0f, -1.0f, 0.0f,
  //         1.0f, 0.0f, 0.0f,
  //         0.0f, 0.0f, 1.0f;
  // raw.segment(0,3) = temp*raw.segment(0,3);
  // raw.segment(3,3) = temp*raw.segment(3,3);
  if(!_wrenchBiasOK && _firstRobotPose)
  {
    Eigen::Vector3f loadForce = _wRb.transpose()*_toolMass*_gravity;
    _wrenchBias.segment(0,3) -= loadForce;
    // _wrenchBias.segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _wrenchBias += raw;
    _wrenchCount++;
    if(_wrenchCount==NB_FT_SAMPLES)
    {
      _wrenchBias /= NB_FT_SAMPLES;
      _wrenchBiasOK = true;
      std::cerr << "[SurfacePuncturing]: Bias: " << _wrenchBias.transpose() << std::endl;
    }
  }

  if(_wrenchBiasOK && _firstRobotPose)
  {
    _wrench = raw-_wrenchBias;
    Eigen::Vector3f loadForce = _wRb.transpose()*_toolMass*_gravity;
    _wrench.segment(0,3) -= loadForce;
    _filteredWrench = _filteredForceGain*_filteredWrench+(1.0f-_filteredForceGain)*_wrench;

    // if(_rawForces.size()<9)
    // {
    //   _rawForces.push_front(_wrench.segment(0,3));
    // }
    // else
    // {
    //   _rawForces.push_front(_wrench.segment(0,3));
    //   _rawForces.pop_back();
    // }

    // if(_filteredForces.size()<9)
    // {
    //   _filteredForces.push_front(_wrench.segment(0,3));
    //   _filteredWrench.segment(0,3) = _wrench.segment(0,3);
    // }
    // else
    // {
    //   if(_rawForces.size()==9)
    //   {      
    //     Eigen::Vector3f filteredForces;
    //     filteredForces = _filter.update(_rawForces);
    //     _filteredWrench.segment(0,3) = filteredForces;
    //   }
    // }
  }
}


void SurfacePuncturing::updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  if(!_firstDampingMatrix)
  {
    _firstDampingMatrix = true;
  }

  _D << msg->data[0],msg->data[1],msg->data[2],
        msg->data[3],msg->data[4],msg->data[5],
        msg->data[6],msg->data[7],msg->data[8];
}

