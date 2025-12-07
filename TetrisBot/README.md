
## Running sequence:
_Note: the install and build folders are git-ignored, so you'll have to do this every time you clone it on a new computer._
_Run colon build and source in the root directory_
### Step 0: enable arm comms
```
ros2 run ur7e_utils enable_comms
```
### Step 1: start perception launch file
```
ros2 launch perception perception.launch.py ar_marker:=<insert arm base arucotag id>
```
If it doesn't seem to register proper base marker id from argument, you can change the default value in src/perception/launch/perception.launch.py:
```
ar_marker_launch_arg = DeclareLaunchArgument(
        'ar_marker',
        # default_value='ar_marker_7'
        default_value='7' <--
    )
```
Then run:
```
ros2 launch perception perception.launch.py ar_marker:=<insert arm base arucotag id>
```

### Step 2: connect the tree to camera
```
ros2 run planning tf --ros-args -p ar_marker:=<insert arm base arucotag id>
```
If it doesn't seem to register proper base marker id, you can change the default value in planning/static_tf_transform.py:
```
#AR Marker parameter
    self.declare_parameter('ar_marker', "7") <--
```
Then run above but without args
### Step 3: run planning.main to start moving boxes
```
ros2 run planning main
```

## Resetting arm (keep enable_comms active)
### To move arm to default pose
```
ros2 run ur7e_utils tuck
```
### To reset arm after estop (and lifting up estop)
```
ros2 run ur7e_utils reset_state
```

## Misc stuff (for debugging purposes):
### Start camera
```
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true rgb_camera.color_profile:=1920x1080x30
```

### Start aruco_recongnition node:
```
ros2 launch ros2_aruco aruco_recognition.launch.py
```
