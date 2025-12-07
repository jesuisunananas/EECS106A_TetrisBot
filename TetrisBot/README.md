

## Running system:
_Note: the install and build folders are git-ignored, so you'll have to do this every time you clone it on a new computer._
_Run colon build and source in the root directory_
### Step 0: run ur7e comms
```
ros2 run ur7e_utils enable_comms
```

### Step 1: launch perception launch file
```
ros2 launch perception perception.launch.py ar_marker:=<insert base link ar marker id>
```

### Step 2: run planning.main
```
ros2 run planning main
```

### Step 3: call service to run packing
```
ros2 service call /run_packing std_srvs/srv/Empty
```

## Misc:

### Reset arm
```
ros2 run ur7e_utils reset_state
```

### Tuck arm
```
ros2 run ur7e_utils tuck
```
