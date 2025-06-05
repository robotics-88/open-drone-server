# simple-drone-server

A REST server that exposes high-level missions and sends drone status to the simple-drone-frontend. Assumes the drone is running open-drone-core for ROS2 capabilities and mission management.

## setup
```
git clone https://github.com/robotics-88/open-drone-server.git
cd open-drone-server
python -m venv .env
source .env/bin/activate
pip install -r requirements.txt
```

## run
```
source .env/bin/activate
uvicorn main:app --host 0.0.0.0 --port 8080
```