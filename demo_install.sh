# create workspace
mkdir ~/ncs_ws/src -p
cd ~/ncs_ws
git clone https://github.com/sharp-rmf/RoMi-H_SI_Empanelment.git
cd RoMi-H_SI_Empanelment
git checkout AMR
touch COLCON_IGNORE
cd ../src
# setup for AMR MiR100
git clone https://github.com/sharp-rmf/fleet_driver_mir.git
# install mir100-client
sudo pip3 install git+https://github.com/osrf/mir100-client.git

cd ..
cp RoMi-H_SI_Empanelment/steps/0-orientation/chart -rf src/

# source path and build
. /opt/ros/foxy/setup.bash
. ~/rmf_demos_ws/install/setup.bash
# To run the fleet driver, use the following command:
colcon build --symlink-install

. ~/ncs_ws/install/setup.bash 
ros2 run fleet_driver_mir fleet_driver_mir ~/ncs_ws/src/chart/mir_fleet_config_cgh_chart.json

