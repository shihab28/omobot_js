

sudo apt-get update
sudo apt-get install python3.6-dev
sudo apt-get install python-pip 
sudo apt-get install python3-pip
python -m pip install --upgrade pip setuptools wheel
python3 -m pip install --upgrade pip setuptools wheel 
sudo apt-get update
sudo apt install openssh-server
sudo apt install openssh-client
sudo apt install sysstat

sudo apt-get install v4l-utils
sudo apt install ubuntu-restricted-extras 
sudo apt-get install ros-melodic-rosserial-arduino
sudo apt-get install -y ros-melodic-serial ros-melodic-joy ros-melodic-joy-teleop ros-melodic-twist-mux ros-melodic-move-base ros-melodic-amcl ros-melodic-map-server ros-melodic-map-laser 

sudo apt-get install blueman # use bluetooth manager if facing trouble to connect controller with jetson, pain and trust the controller

pip3 install  python-apt ubuntu-drivers-common
pip3 install Jetson.GPIO jetson-stats Adafruit-GPIO Adafruit-SSD1306  adafruit-circuitpython-mpu6050       # for jetson nano user

pip3 install ssh-import-id  systemd-python system-service
pip3 install pyserial pyserial-asyncio asyncio

# create symlink for the serial ports. ArduinoMot indicates the arduino connected for motor control, ArduinoEnc indicates the arduino connected for encoder feedback
# sudo nano /etc/udev/rules.d/99-serial-ports.rules 
# check the relevant parameters using  "udevadm info --name=/dev/ttyACM0 --attribute-walk" and "udevadm info --name=/dev/ttyACM1 --attribute-walk" for the two arduino
# And paste the following lines for two arduinos.
        # SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="8057", ATTRS{serial}=="19E31D7550304D48502E3120FF122217", SYMLINK+="ArduinoMot"
        # SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="005e", ATTRS{serial}=="06C63450187B8A92", SYMLINK+="ArduinoEnc"


