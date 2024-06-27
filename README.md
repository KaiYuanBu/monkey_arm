
# monkey_arm

ROS2 packages for MonKey Arm module.


## Installation

Python CAN Library:
```bash
pip install python-can
```

Python CAN Library (with Serial commumication):
```bash
pip install python-can[serial]
```
    
## Docker Deployment

### Windows
To build docker image:
```bash
docker build -t monkey/humble/arm .
```

To run docker image:
```bash
docker run --rm -it --name monkey-arm --user monkey -v $PWD/.:/home/monkey/arm_ws/src -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 monkey/humble/arm bash
```

### Linux
```bash
docker run --rm -it --name monkey-arm --user monkey -v $PWD/.:/home/monkey/arm_ws/src -e DISPLAY=$DISPLAY -e LIBGL_ALWAYS_INDIRECT=0 monkey/humble/arm bash
```

## Authors

- [@LocoHao](https://github.com/LocoHao)

