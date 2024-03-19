

sudo apt-get update
sudo apt-get install python3.6-dev
sudo apt-get install python-pip 
sudo apt-get install python3-pip
python -m pip install --upgrade pip setuptools wheel
python3 -m pip install --upgrade pip setuptools wheel 
sudo apt-get update
 cython cpython opencv-python numpy pandas matplotlib pillow
sudo apt install openssh-server
sudo apt install openssh-client
sudo apt install sysstat
sudo apt-get install v4l-utils
sudo apt install -y ubuntu-restricted-extras
sudo apt-get install -y blueman # use bluetooth manager if facing trouble to connect controller with jetson, pain and trust the controller

sudo apt-get install -y ros-melodic-rosserial-arduino
sudo apt-get install -y ros-melodic-serial ros-melodic-joy ros-melodic-joy-teleop ros-melodic-twist-mux ros-melodic-move-base ros-melodic-amcl ros-melodic-map-server ros-melodic-map-laser 
sudo apt-get install -y ros-melodic-aruco-ros ros-melodic-aruco-detect ros-melodic-image-pipeline 


pip3 install  python-apt ubuntu-drivers-common
pip3 install Jetson.GPIO jetson-stats Adafruit-GPIO Adafruit-SSD1306  adafruit-circuitpython-mpu6050       # for jetson nano user
pip3 install cython cpython numpy pandas matplotlib pillow
pip2 install cython cpython numpy pandas matplotlib pillow
pip3 install ssh-import-id  systemd-python system-service
pip3 install pyserial pyserial-asyncio asyncio


############################################Install Lidar-LD19 ROS Driver
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros.git 
############################################

############################################Install IMU-6050 ROS Driver
git clone https://github.com/OSUrobotics/mpu_6050_driver.git
############################################

############################################Install Pytorch 1.8.0 Using Following
#         pip3 install torch==1.8.2 torchvision==0.9.2 torchaudio==0.8.2 --extra-index-url https://download.pytorch.org/whl/lts/1.8/cu102
#         https://qengineering.eu/install-pytorch-on-jetson-nano.html
############################################


############################################ create symlink for the serial ports. arduinoMot indicates the arduino connected for motor control, arduinoEnc indicates the arduino connected for encoder feedback
# sudo nano /etc/udev/rules.d/99-serial-ports.rules 
# check the relevant parameters using  "udevadm info --name=/dev/ttyACM0 --attribute-walk" and "udevadm info --name=/dev/ttyACM1 --attribute-walk" for the two arduino
# And paste the following lines for two arduinos.
        # SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="8057", ATTRS{serial}=="19E31D7550304D48502E3120FF122217", SYMLINK+="ArduinoMot"
        # SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="005e", ATTRS{serial}=="06C63450187B8A92", SYMLINK+="ArduinoEnc"
############################################


############################################ for cuda10.2 and CudNN ############################################
#sudo apt search cuda
# sudo apt-get install cuda-tools-10-2 cuda-libraries-10-2


