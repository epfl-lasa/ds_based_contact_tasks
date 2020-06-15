#include "SurfaceCutting.h"
#include "Utils.h"
#include <numeric>

SurfaceCutting* SurfaceCutting::me = NULL;

SurfaceCutting::SurfaceCutting(ros::NodeHandle &n, double frequency, std::string fileName, float targetVelocity, float targetForce):
  _nh(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _fileName(fileName),
  _targetVelocity(targetVelocity),
  _targetForce(targetForce)
{
  me = this;

  _sim = false;

  _gravity << 0.0f, 0.0f, -9.80665f;
 
  // Version small f/t sensor
  _toolComPositionFromSensor<< 0.0f,0.0f,0.035f;
  _toolOffsetFromEE << 0.0f, -0.028, 0.132f;
  _toolOffsetFromEE << 0.0f, 0.0f, 0.1286f+0.015f;
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

  _xa[0] << -0.4f, -0.5f, 0.4f-_toolOffsetFromEE(2);
  _xa[1] << -0.4f, -0.4f, 0.202f-_toolOffsetFromEE(2);
  _xa[2] << -0.4f, -0.1f, 0.202f-_toolOffsetFromEE(2);
  _xa[3] << -0.4f, 0.0f, 0.4f-_toolOffsetFromEE(2);
  // _xs << 0.0f,0.0f,0.136f-_toolOffsetFromEE(2);
  _xs << -0.656f,-0.156f,0.117f+0.01f-_toolOffsetFromEE(2);
  _attractorID = 0;
  _xs << -0.5398f,-0.0199f,0.256f-_toolOffsetFromEE(2);


  Eigen::VectorXf x,y, cumsum;
  float xmin = -2.0;
  float xmax = 2.0f;
  float step = 0.00005f;
  float p = 0.1f;
  x.setLinSpaced(int((xmax-xmin)/step+1),xmin,xmax);
  cumsum.resize(x.size());
  y.resize(x.size());

  cumsum.setConstant(0.0f);
  for(int k = 0; k < x.size(); k++)
  {
    y(k) = x(k)*std::sin(2*std::pow(x(k),2.0f));
  }
 
  for(int k = 0; k < x.size(); k++)
  {
    if(k>0)
    {
      Eigen::Vector2f point, prevPoint;
      point << x(k), y(k);
      prevPoint << x(k-1), y(k-1);
      cumsum(k)=cumsum(k-1)+(point-prevPoint).norm();
    }
  }

  std::vector<int> id;
  id.push_back(0);
  uint32_t count = 1;
  for(int k = 0; k < x.size(); k++)
  {
    if(cumsum(k)>count*p)
    {
      count++;
      id.push_back(k);
    }
  }

  _xa2;
  _xa2.resize(id.size(),3);
  for(int k = 0 ; k < _xa2.rows(); k++)
  {
    _xa2.row(k) << 0.1f*y(id[k]),0.1f*x(id[k]),0.064;
  }

  _xa2.col(0).array()-=0.55f;
  // _xa2.col(1).array()-=0.3f;

  std::cerr << _xa2 << std::endl;

  _filteredForceGain = 0.9;
  _ke = 0.0f;
  _deltac = 0.0f;

  _deltad = 0.005f;

  // _filter.init(1.0f, -1.189f, 0.603f, 1.0f, 0.757f, 1.0f, 
  //              1.0f, -0.895f, 0.228f, 1.0f, 1.712f, 1.0f, 0.15f*0.089f);

  Eigen::VectorXf coeff;
  coeff.resize(9);
  coeff << -0.0013f, 0.0311f, 0.1309f, 0.2645f, 0.3289f, 0.2645f, 0.1309f, 0.0311f, -0.0013f;

  _filter.init(coeff);  

  _dNormalForce = 0.0f;
  _prevDNormalForce = 0.0f;
  _ddNormalForce = 0.0f;
  _prevDDNormalForce = 0.0f;
  _picDetected = false;

  std::cerr << "AAAAAAAAAAAAAAAAAAAA" << std::endl;
  _gp1 = new GP_32(1,1);
  _gp1->load<serialize::BinaryArchive>("gp_model_1", false);
  _gp2 = new GP_52(1,1);
  _gp2->load<serialize::BinaryArchive>("gp_model_2", false);
  _gp3 = new GP_52(1,1);
  _gp3->load<serialize::BinaryArchive>("gp_model_3", false);

  std::cerr << "BBBBBBBBBBBBBBBBBBBBBBB" << std::endl;

  // Eigen::VectorXd mu;
  // double sigma;
  
  // double t1 = ros::Time::now().toSec();
  // std::tie(mu, sigma) = _gp->query(tools::make_vector(0.02f));
  // // mu = _gp->mu(tools::make_vector(0.02f));
  // double t2 = ros::Time::now().toSec();

  // std::cerr << t2-t1 << " " << mu(0) << std::endl;

  _compensation = 0.0f;

  // _filter.init(1.0f, -0.895f, 0.228f, 1.0f, 1.712f, 1.0f,
  //              1.0f, -1.189f, 0.603f, 1.0f, 0.757f, 1.0f, 0.15f*0.089f);
}


bool SurfaceCutting::init()
{
  // Subscriber definitions
  _subRobotPose = _nh.subscribe("/lwr/ee_pose", 1, &SurfaceCutting::updateRobotPose, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist = _nh.subscribe("/lwr/ee_vel", 1, &SurfaceCutting::updateRobotTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor = _nh.subscribe("/ft_sensor/netft_data", 1, &SurfaceCutting::updateRobotWrench, this, ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix = _nh.subscribe("/lwr/joint_controllers/passive_ds_damping_matrix", 1, &SurfaceCutting::updateDampingMatrix,this,ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist = _nh.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation = _nh.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubFilteredWrench = _nh.advertise<geometry_msgs::WrenchStamped>("SurfaceCutting/filteredWrench", 1);
  _pubNormalForce = _nh.advertise<std_msgs::Float32>("SurfaceCutting/normalForce", 1);
  _pubDepth = _nh.advertise<std_msgs::Float32>("SurfaceCutting/depth", 1);


  signal(SIGINT,SurfaceCutting::stopNode);


  ROS_INFO("[SurfaceCutting]: Filename: %s", _fileName.c_str());

  _outputFile.open(ros::package::getPath(std::string("ds_based_contact_tasks"))+"/data_cutting/"+_fileName+".txt");

  if(!_nh.getParamCached("/lwr/joint_controllers/ds_param/damping_eigval0",_d1))
  {
    ROS_ERROR("[SurfaceCutting]: Cannot read first eigen value of passive ds controller");
    return false;
  }

  if(!_outputFile.is_open())
  {
    ROS_ERROR("[SurfaceCutting]: Cannot open output data file, the data_cutting directory might be missing");
    return false;
  }

  if(_targetVelocity>0.0f)
  {
    ROS_INFO("[SurfaceCutting]: Target velocity: %f", _targetVelocity);
  }
  else
  {
    ROS_ERROR("[SurfaceCutting]: Target velocity should be positive");
    return false;
  }

  if(_targetForce>0.0f)
  {
    ROS_INFO("[SurfaceCutting]: Target force: %f", _targetForce);
  }
  else
  {
    ROS_ERROR("[SurfaceCutting]: Target force should be positive");
    return false;
  }

  return true;
}


void SurfaceCutting::run()
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


void SurfaceCutting::stopNode(int sig)
{
  me->_stop = true;
}


bool SurfaceCutting::allSubscribersOK()
{
  bool status = _firstRobotPose && _firstRobotTwist && (_sim || _wrenchBiasOK);

  if(!status)
  {
    std::cerr << "p: " << (int) _firstRobotPose << " t: " << (int) _firstRobotTwist << " w: " << (int) _wrenchBiasOK << std::endl;
  }
  return status;     
}


void SurfaceCutting::computeCommand()
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


void SurfaceCutting::updateSurfaceInformation()
{
  _planeNormal << 0.0f, 0.0f, 1.0f;    
  _wRs.setIdentity();

  _xProj = _x;
  _xProj(2) = (-_planeNormal(0)*(_xProj(0)-_xa[1](0))-_planeNormal(1)*(_xProj(1)-_xa[1](1))+_planeNormal(2)*_xa[1](2))/_planeNormal(2);
  // _xProj(2) = (-_planeNormal(0)*(_xProj(0)-_xa2(0,0))-_planeNormal(1)*(_xProj(1)-_xa2(0,1))+_planeNormal(2)*_xa2(0,2))/_planeNormal(2);

  // Compute _n = normal vector pointing towards the surface
  _n = -_planeNormal;
      
  // Compute signed normal distance to the plane
  _normalDistance = (_xProj-_x).dot(_n);

  if(_normalDistance < 0.0f)
  {
    _normalDistance = 0.0f;
  }
 
  std::cerr << "[SurfaceCutting]: Normal distance: " << _normalDistance << " Normal vector: " << _n.transpose() << std::endl;    

  // Compute normal force
  Eigen::Vector3f F = _filteredWrench.segment(0,3);
  Eigen::Vector3f w = _filteredWrench.segment(3,3);
  std::cerr << _filteredWrench.transpose() << std::endl;
  // std::cerr << "Force: " << (_wRb*F).transpose() << std::endl;
  _prevNormalForce = _normalForce;
  _normalForce = _n.dot(-_wRb*F);
  // std::cerr << _normalForce << std::endl;
  Eigen::Vector3f temp = _wRb*w;
  // std::cerr << "Torque: " << temp.transpose() << std::endl;
  // std::cerr << "bou: " << temp(0)/0.021 << std::endl;

  _depth = _x(2)-_xs(2);
  _normalDistance = _depth;

  std::cerr << "[SurfaceCutting]: Depth: " << _depth << " Normal force: " << _normalForce << std::endl;    

  // Eigen::Vector3f temp2;
  // temp2 = temp.cross(_wRb*_toolOffsetFromEE)/_toolOffsetFromEE.squaredNorm();
  // std::cerr << "Force from torque" << temp2.transpose() << std::endl;

}


void SurfaceCutting::updateContactState()
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


void SurfaceCutting::computeNominalDS()
{
  if((_xa[_attractorID]-_x).norm()<0.025)
  {
    _attractorID++;
    if(_attractorID>NB_ATTRACTORS-1)
    {
      _attractorID = 0;  
    }
  }
  _fx = 4.0f*(_xa[_attractorID]-_x);



  // if(_contactState==CONTACT)
  // {
  //   _fx.setConstant(0.0f);
  // }
  // else
  {
    _fx = 2.0f*(_xs-_x);
  }

  Eigen::Vector3f xa;
  xa = _xd;
  xa(2) = _xs(2)-0.045f;
  _fx = (xa-_x);
  _fx = Utils<float>::bound(_fx,0.03f);
  
  if(_contactState==CONTACT)
  {
    float adaptationRate = 5.0f;
    _deltav += _dt*adaptationRate*_n.transpose()*(_fx-_v);
    // std::cerr << _n.transpose()*(_fx-_v) << std::endl;
    // std::cerr << _fx.norm()/2 << std::endl;

   _deltav = Utils<float>::bound(_deltav,0.0f,0.2f);
   _fx+=_deltav*_n;
   

    Eigen::VectorXd result[3];
    Eigen::Vector3f sumError;
    

    for(int k = 0; k < 3; k++)
    {
      if(k==0)
      {
        result[k] = _gp1->mu(tools::make_vector(-_depth));
      }
      else if(k==1)
      {
        result[k] = _gp2->mu(tools::make_vector(-_depth));      
      }
      else
      {
        result[k] = _gp3->mu(tools::make_vector(-_depth));      
      }
      _gpError[k].push_front(std::fabs(result[k](0)-_normalForce));
      if(_gpError[k].size()>10)
      {
        _gpError[k].pop_back();
      }

      Eigen::VectorXf temp;
      temp.resize(_gpError[k].size());
      for(int m = 0; m < temp.size(); m++)
      {
        temp(m) = _gpError[k][m];
      }
      
      sumError(k) = temp.sum();
    }

    Eigen::MatrixXf::Index id;
    sumError.minCoeff(&id);
    std::cerr << "Error: " << sumError(0) << " " << sumError(1) << " " << sumError(2) << std::endl;

    std::cerr << "Predicted model: " << id << std::endl;



     // _compensation = result(0);
     _compensation = 0.0f;
     // std::cerr << "compensation: " << _compensation  << std::endl;
     if(_compensation >16)
     {
      _compensation  = 16.0f;
     } 
     // _fx+=(_compensation/_d1)*_n; 
      // std::cerr << "deltav: " << _deltav << " v:" << _n.dot(_v) << std::endl;
   
    _prevDNormalForce = _dNormalForce;
    _dNormalForce = (_normalForce-_prevNormalForce)/_dt;
    _prevDDNormalForce = _ddNormalForce;
    _ddNormalForce = _dNormalForce-_prevDNormalForce/_dt;

    if(_ddNormalForce < 0.0f && _prevDDNormalForce > 0.0f)
    {
      _positivePicDForce.push_back(_dNormalForce);
      _positivePicTime.push_back(ros::Time::now().toSec());
    }
    else if(_ddNormalForce > 0.0f && _prevDDNormalForce < 0.0f && _dNormalForce < 0.0f)
    {
      _negativePicDForce.push_back(_dNormalForce);
      _negativePicTime.push_back(ros::Time::now().toSec()); 
      if(!_positivePicDForce.empty())
      {
        float deltaPic = _negativePicDForce.back()-_positivePicDForce.back();
        float deltaTime = _negativePicTime.back()-_positivePicTime.back();
        if(deltaPic < -12.0f && deltaTime < 0.1f)
        {
          _picDetected = true;
        }
      }
    }
    
    std::cerr << "Pic detected: " << (int) _picDetected << std::endl;  
    if(_picDetected)
    {
      _fx.setConstant(0.0f);
    }
    std::cerr << xa(2)-_x(2) << std::endl;
  }




  // float adaptationRateKe = 100.0f;
  // float adaptationRateDeltac = 0.000001f;
  // if(_contactState==CONTACT)
  // {
  //   float dke;
  //   dke = _ke*(-_depth)-_normalForce;
  //   _ke += -_dt*adaptationRateKe*dke;
  //   _ke = Utils<float>::bound(_ke,0.0f,3500.0f);

  //   float ddeltac;
  //   ddeltac = -_ke*(_ke*_deltad-_normalForce);
  //   _deltac += -_dt*adaptationRateDeltac*ddeltac;
  //   _deltac = Utils<float>::bound(_deltac,-_deltad,0.01f);
  // }

  // std::cerr << "Stiffness: " << _ke << " Deltac: " << _deltac <<  std::endl;
 


  // if((_xa2.row(_attractorID).transpose()-_x).norm()<0.025)
  // {
  //   _attractorID++;
  //   if(_attractorID>_xa2.rows()-1)
  //   {
  //     _attractorID = 0;  
  //   }
  // }

  // float temp;
  // Eigen::VectorXf::Index index;
  // temp = (_x.transpose().replicate(_xa2.rows(),1)-_xa2).rowwise().norm().array().minCoeff(&index);

  // _xc = _xa2.row(index).transpose();
  // if(index<_xa2.rows()-2)
  // {
  //   index+=2;
  // }
  // else
  // {
  //   index = _xa2.rows()-1;
  // }

  // _xd = _xa2.row(index).transpose();
  // Eigen::Matrix3f gain;
  // // gain.setConstant(0.0f);
  // if(index==_xa2.rows()-1)
  // {
  //   gain(0,0) = 4.0f;
  //   gain(1,1) = 4.0f;
  //   gain(2,2) = 4.0f;    
  // }
  // else
  // {
  //   gain(0,0) = 10.0f;
  //   gain(1,1) = 10.0f;
  //   gain(2,2) = 10.0f;        
  // }
  // _fx = gain*(_xa2.row(index).transpose()-_x);

  // if(index<_xa2.rows()-1)
  // {
  //   _fx = 0.2f*_fx.normalized();
  // }
  // _fx = 0.2f*_fx.normalized();
  // _fx = Utils<float>::bound(_fx,0.15f);

  std::cerr << "[SurfaceCutting]: "<<  _attractorID << " " << (_xa[_attractorID]-_x).norm() << std::endl;
  // std::cerr << "[SurfaceCutting]: "<<  index << " " << (_xa2.row(index).transpose()-_x).norm() << std::endl;
}


void SurfaceCutting::computeDesiredContactForceProfile()
{
  if(_contactState==CONTACT)// && _attractorID == 2)
  {
    // _Fd = _targetForce;
    _Fd = std::max(_ke*(_deltad+_deltac),5.0f);

  }
  else if(_contactState==CLOSE_TO_CONTACT)//  && _attractorID == 2)
  {
    _Fd = 5.0f;
    // _Fd = 0.0f;
  }
  else
  {
    _Fd = 0.0f;
  }

  std::cerr << "[SurfaceCutting]: Fd: " << _Fd << " c: " << _c << " Normal force: " << _normalForce << std::endl;
}

void SurfaceCutting::computeModulationTerms()
{
  _fxn = (_Fd/_d1)*_n;
  _fxn.setConstant(0.0f);
}


void SurfaceCutting::computeModulatedDS()
{

  // Compute corrected nominal DS and modulation terms
  _fxp = _fx;
  _fxnp = _fxn;
  // _fxp.setConstant(0.0f);
  // _fxnp = (15.0f/_d1)*_n;
  _fxnp.setConstant(0.0f);
  // Compute modulated DS
  _vd = _fxp+_fxnp;

  // Bound modulated DS for safety
  _vd = Utils<float>::bound(_vd,0.2f);

  std::cerr << "[SurfaceCutting]: fx: " << _fx.norm() << std::endl;


  std::cerr << "[SurfaceCutting]: vd: " << _vd.norm() << " v: " << _v.norm() <<std::endl;
}


void SurfaceCutting::computeDesiredOrientation()
{
  // Compute rotation error between current orientation and plane orientation using Rodrigues' law

  _qd << 0.0f, 0.0f, 1.0f, 0.0f;
  Eigen::Matrix3f Rd;
  // float theta = 20.0f*M_PI/180.0f;
  float theta = 0.0f*M_PI/180.0f;
  
  Rd << -1.0f, 0.0f, 0.0f,
          0.0f, std::cos(theta), std::sin(theta),
          0.0f, std::sin(theta), -std::cos(theta);

  _qd = Utils<float>::rotationMatrixToQuaternion(Rd);


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

void SurfaceCutting::logData()
{
  Eigen::Vector3f F, Ffiltered;
  F = -_wRb*_wrench.segment(0,3);
  Ffiltered = -_wRb*_filteredWrench.segment(0,3);
  _outputFile << ros::Time::now() << " "
              << _x.transpose() << " "
              << _v.transpose() << " "
              << _normalForce << " "
              << Ffiltered.transpose() << " "
              << F.transpose() << " "
              << _c << " "
              << _depth << " "
              << _prevNormalForce << " " 
              << _prevDNormalForce << " " 
              << _dNormalForce << " " 
              << _prevDDNormalForce << " " 
              << _ddNormalForce << " " 
              << (int) _picDetected << " " 
              << _compensation << std::endl;
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


void SurfaceCutting::publishData()
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

  msg.data = _depth;
  _pubDepth.publish(msg);
}


void SurfaceCutting::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg)
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


void SurfaceCutting::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg)
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


void SurfaceCutting::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::Matrix<float,6,1> raw;
  // raw(0) = msg->wrench.force.x;
  // raw(1) = msg->wrench.force.y;
  // raw(2) = msg->wrench.force.z;
  // raw(3) = msg->wrench.torque.x;
  // raw(4) = msg->wrench.torque.y;
  // raw(5) = msg->wrench.torque.z;
  raw(0) = msg->wrench.force.y;
  raw(1) = -msg->wrench.force.x;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.y;
  raw(4) = -msg->wrench.torque.x;
  raw(5) = msg->wrench.torque.z;

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
      std::cerr << "[SurfaceCutting]: Bias: " << _wrenchBias.transpose() << std::endl;
    }
  }

  if(_wrenchBiasOK && _firstRobotPose)
  {
    _wrench = raw-_wrenchBias;
    Eigen::Vector3f loadForce = _wRb.transpose()*_toolMass*_gravity;
    _wrench.segment(0,3) -= loadForce;
    // _wrench.segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _filteredWrench = _filteredForceGain*_filteredWrench+(1.0f-_filteredForceGain)*_wrench;

    if(_rawForces.size()<9)
    {
      _rawForces.push_front(_wrench.segment(0,3));
    }
    else
    {
      _rawForces.push_front(_wrench.segment(0,3));
      _rawForces.pop_back();
    }

    if(_filteredForces.size()<9)
    {
      _filteredForces.push_front(_wrench.segment(0,3));
      _filteredWrench.segment(0,3) = _wrench.segment(0,3);
    }
    else
    {
      if(_rawForces.size()==9)
      {      
        // std::cerr << "a" << std::endl;
        Eigen::Vector3f filteredForces;
        filteredForces = _filter.update(_rawForces);
        _filteredWrench.segment(0,3) = filteredForces;
        // std::cerr << filteredForces << std::endl;
        // std::cerr << _rawForces[0].transpose() << std::endl;
        // std::cerr << _filteredForces[0].transpose() << std::endl;
      }
    }
  }
  // std::cerr << _wrench.transpose() << std::endl;
  // std::cerr << _wrenchBias.transpose() << std::endl;
}


void SurfaceCutting::updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  if(!_firstDampingMatrix)
  {
    _firstDampingMatrix = true;
  }

  _D << msg->data[0],msg->data[1],msg->data[2],
        msg->data[3],msg->data[4],msg->data[5],
        msg->data[6],msg->data[7],msg->data[8];
}

