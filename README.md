# rrfmgz

Random Robot Forest Motion GaZebo (name will be changed in the future!)

# TODO project

- [ ] rename repo

# TODO simulation

- [x] create lehrforst with selection of trees
- [x] create robot model with bumper and sensors
- [x] ros gz bridge etc.
- [x] build for test simulation with test algorithm
- [x] build simulation border
- [x] set border with coordinates in software not with shape in gz
- [x] build an "advanced algorithm"
- [ ] detect crash when driving backwards (just for tracking if robot would be damaged)
- [ ] (and/or) build back bumper and adapt in software
- [x] record map with collisions (!)
- [x] improve tracking and recording
- [x] implement angle of tree inclination in model
- [x] simulate imu
- [x] simulate GPS (gazebo position)
- [x] organize into ros packages
- [x] make real launch files
- [x] make simulation pretty
- [x] live plot of robot position (in `plot_robot_path.py` under scripts)
- [ ] maybe integrate real bumper (or load cell) into simulation for testing and calibrating

# TODO real robot

Integrate the sensor and robot ros control into the platform computer (currently a Raspberry Pi 5).

- [x] GNSS
- [ ] IMU
- [ ] integrate integrated robot odometry (requires repaired robot)
- [ ] stereo microphone (ReSpeaker 2-Mic Pi HAT V1.0)
- [ ] and setup Pi Zero W for it (if needed)
- [ ] environment sensor (BME688, this should be the better sensor)
- [ ] (and/or) environment sensor (SHT31-D)
- [ ] thermal camera (MLX90640-D55)
- [x] Ouster LiDAR
- [ ] RoboSense LiDAR (maybe)
- [x] Intel Realsense (Stereo Camera)
- [ ] ZED Stereo Camera (if possible) -- Džemail
- [ ] test implementation of a Gauge Load Cell (optional, playing around with it)
- [ ] setup Arduinos for bumper (for the start with the test implementation above)
- [ ] THE BUMPER -- wait until first segment is build by Florian
- [ ] repair SCOUT -- Florian (thanks)
- [ ] SteamDeck controller for all sensors and robot controls

# TODO RIEGL (side) project

Project with Christoph Gollob and his team about combining his LiDAR with our robots (big bunker probably).
The main goal of the combination is planning and scanning a predefined path (or a recorded path) in the forest
where on fixed locations a LiDAR scan from the RIEGL sensor will be made (stop and go).
Autonomous forest navigation is a big keypoint here, even if the path is predefined (navigating everchanging
forest is difficult).

- [ ] Get big Bunker working with ros and the Pi5 (CAN bus)
- [x] record movement and play back (rosbag)
- [ ] navigate robot just with odom (see `move_robot_wo_tf.py` in scripts)
- [ ] make system for planning a path with fixed scann positions
- [ ] and driving that path