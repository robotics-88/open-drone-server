# simple-drone-server

A REST server that exposes high-level missions and sends drone status to the simple-drone-frontend. Assumes the drone is running open-drone-core for ROS2 capabilities and mission management, and that there is a local ROS2 installation. 

## setup
```
git clone https://github.com/robotics-88/open-drone-server.git
cd open-drone-server
python3 -m venv .env --system-site-packages
source .env/bin/activate
pip install -r requirements.txt
```

## run
```
source .env/bin/activate
source /opt/ros/$ROS_DISTRO/setup.bash
uvicorn main:app --host 0.0.0.0 --port 8080
```