#helo

##Install (with CUDA):
```
conda create -n TetrisBot python=3.9 numpy
conda activate TetrisBot
pip install tensorflow[and-cuda]
pip install pytest
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install pybullet
```

###To run tests (from inside packing dir):
```
pytest
```

###To retrain (generates new packing/critic.pt and packing/policy.pt):
```
python packing/main.py --mode train
```

###To run visualization (no training):
```
python packing/main.py --mode eval
```

Note on colon build and source (in the root directory)
Note: the install and build folders are git-ignored, so you'll have to do this every time you clone it on a new computer.

##Running ros2_aruco:
###Step 1: start camera
```
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true rgb_camera.color_profile:=1920x1080x30
```

###Step 2: launch the aruco_recongnition node:
```
ros2 launch ros2_aruco aruco_recognition.launch.py
```

##Running perception:
All the code are in ar_tag_identity.py 

###Step 1: start camera
```
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true rgb_camera.color_profile:=1920x1080x30
```

###Step 2: start the base-link tree
```
ros2 run ur7e_utils enable_comms
```

###Step 3: connect the tree to camera
```
ros2 run planning tf
```
###Step 4: launch the node
```
ros2 launch perception perception.launch.py
```


