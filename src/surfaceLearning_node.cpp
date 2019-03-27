#include "SurfaceLearning.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "surface_learning");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
  std::string fileName;

  SurfaceLearning::Mode mode;

  float C = 100.0f, sigma = 0.2f, epsilonTube = 0.015f;

  bool gneerateDataset = false;
  bool addDataOnSurface = false;


  if(argc >= 4) 
  {
    fileName = std::string(argv[1]);

    if(std::string(argv[2]) == "-m" && std::string(argv[3]) == "c")
    {
      mode = SurfaceLearning::Mode::COLLECTING_DATA;
    }
    else if(std::string(argv[2]) == "-m" && std::string(argv[3]) == "l" && argc == 14)
    {
      mode = SurfaceLearning::Mode::LEARNING;
      
      if(std::string(argv[4]) == "-c")
      {
        C = atof(argv[5]);
      }
      else
      {
        ROS_ERROR("Wrong C arguments, the command line arguments should be: fileName -m(mode) l(learning) -c(C) C -s(sigma) s -e(epsilon tube) e -g(generate dataset) y(yes) or n(no) -u(use data on surface) y(yes) or n(no)");
        return 0;
      }

      if(std::string(argv[6]) == "-s")
      {
        sigma = atof(argv[7]);
      }
      else
      {
        ROS_ERROR("Wrong sigma arguments, the command line arguments should be: fileName -m(mode) l(learning) -c(C) C -s(sigma) s -e(epsilon tube) e -g(generate dataset) y(yes) or n(no) -u(use data on surface) y(yes) or n(no)");
        return 0;
      }

      if(std::string(argv[8]) == "-e")
      {
        epsilonTube = atof(argv[9]);
      }
      else
      {
        ROS_ERROR("Wrong epsilon tube arguments, the command line arguments should be: fileName -m(mode) l(learning) -c(C) C -s(sigma) s -e(epsilon tube) e -g(generate dataset) y(yes) or n(no) -u(use data on surface) y(yes) or n(no)");
        return 0;
      }

      if(std::string(argv[10]) == "-g" && std::string(argv[11]) == "y")
      {
        gneerateDataset = true;
      }
      else if(std::string(argv[10]) == "-g" && std::string(argv[11]) == "n")
      {
        gneerateDataset = false; 
      }
      else
      {
        ROS_ERROR("Wrong generate dataset arguments, the command line arguments should be: fileName -m(mode) l(learning) -c(C) C -s(sigma) s -e(epsilon tube) e -g(generate dataset) y(yes) or n(no) -u(use data on surface) y(yes) or n(no)");
        return 0;
      }

      if(std::string(argv[12]) == "-u" && std::string(argv[13]) == "y")
      {
        addDataOnSurface = true;
      }
      else if(std::string(argv[12]) == "-u" && std::string(argv[13]) == "n")
      {
        addDataOnSurface = false; 
      }
      else
      {
        ROS_ERROR("Wrong add data on surface arguments, the command line arguments should be: fileName -m(mode) l(learning) -c(C) C -s(sigma) s -e(epsilon tube) e -g(generate dataset) y(yes) or n(no) -u(use data on surface) y(yes) or n(no)");
        return 0;
      }
    }
    else if(std::string(argv[2]) == "-m" && std::string(argv[3]) == "l" && argc != 14)
    {

        ROS_ERROR("Wrong learning mode arguments, the command line arguments should be: fileName -m(mode) l(learning) -c(C) C -s(sigma) s -e(epsilon tube) e -g(generate dataset) y(yes) or n(no) -u(use data on surface) y(yes) or n(no)");
        return 0;  
    }
    else if(std::string(argv[2]) == "-m" && std::string(argv[3]) == "t")
    {
      mode = SurfaceLearning::Mode::TESTING;
    }
    else
    {
      ROS_ERROR("Wrong arguments, the command line arguments should be in learning mode: fileName -m(mode) l(learning) -c(C) C -s(sigma) s -e(epsilon tube) e -g(generate dataset) y(yes) or n(no) -u(use data on surface) y(yes) or n(no)");
      ROS_ERROR("Wrong arguments, the command line arguments should be in collecting or testing mode: fileName -m(mode) c(collecting data) or t (testing) -c(C) C -s(sigma) s -e(epsilon tube) e -g(generate dataset) y(yes) or n(no) -u(use data on surface) y(yes) or n(no)");
      return 0;
    }
  }
  else
  {
    ROS_ERROR("Wrong arguments, the command line arguments should be in learning mode: fileName -m(mode) l(learning) -c(C) C -s(sigma) s -e(epsilon tube) e -g(generate dataset) y(yes) or n(no) -u(use data on surface) y(yes) or n(no)");
    ROS_ERROR("Wrong arguments, the command line arguments should be in collecting or testing mode: fileName -m(mode) c(collecting data) or t (testing) -c(C) C -s(sigma) s -e(epsilon tube) e -g(generate dataset) y(yes) or n(no) -u(use data on surface) y(yes) or n(no)");
    return 0;
  }

  SurfaceLearning surfaceLearning(n,frequency,fileName,mode,C,sigma,epsilonTube,gneerateDataset,addDataOnSurface);

  if (!surfaceLearning.init()) 
  {
    return -1;
  }
  else
  {
    surfaceLearning.run();
  }

  return 0;
}

