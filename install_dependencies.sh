tput setaf 6
echo "#################"
echo "# Install Eigen #"
echo "#################"
tput sgr0
sudo apt install libeigen3-dev
tput setaf 6
echo "##########################################################"
echo "# Install lib ncurses needed by the kuka-lwr-ros package #"
echo "##########################################################"
tput sgr0
sudo apt-get install libncurses5-dev
tput setaf 6
echo "########################################################"
echo "# Install lib armadillo needed for the SVMGrad package #"
echo "########################################################"
tput sgr0
sudo apt-get install libarmadillo-dev
tput setaf 6
echo "################################"
echo "# Get all package dependencies #"
echo "################################"
tput sgr0
sudo wstool init
sudo wstool merge ds_based_contact_tasks/dependencies.rosinstall
sudo wstool up
sudo wstool merge kuka-lwr-ros/dependencies.rosinstall
sudo wstool up
tput setaf 6
echo "##########################################################"
echo "# Compile libsvm and add executables to environment path #"
echo "##########################################################"
tput sgr0
cd libsvm
make
export PATH=$PATH:$(pwd)
cd ../..
tput setaf 6
echo "#####################"
echo "# Make all packages #"
echo "#####################"
tput sgr0
catkin_make
tput setaf 6
echo "###########################"
echo "# Create data directories #"
echo "###########################"
tput sgr0
mkdir src/ds_based_contact_tasks/data_grasping
mkdir src/ds_based_contact_tasks/data_polishing
mkdir src/ds_based_contact_tasks/data_surface
tput setaf 6
echo "#################################"
echo "# Everything done hopefully !!! #"
echo "#################################"
tput sgr0