# jevois_aruco
* clone repo into your ```catkin_ws```
```bash
 git clone https://github.com/mzahana/jevois_aruco.git
 ```
 * Make sure that you have ```Pyserial``` python package
 * compile ```catkin_ws``` using ```catkin build```
 
 # Usage
 * Flash latest JeVois firmware on an SD card
 * Replace the ```initscript.cfg``` file that is on the SD card by the file included in this repo ```config/initscript.cfg```
 * Insert the SD card into JeVois camera
 * Connect your camera through USB (USB 3 is recommended to have enough power)
 * Modify the ```config/setup.yaml``` file to have, for example, the right device address
 * Launch the script
 ```bash
 roslaunch jevois_aruco read_tags.launch
 ```
 * Echo the topic ```/tags``` to see the tags positions
 
 # Notes
 * The orientation of the tags is currently not provided. Only the 3D position
 * Make sure the you set the paramter ```markerlen``` in the ```initscript.cfg``` correctly in order to get good estimates of the positions. This is the actual length of the arker side in milimeters.
