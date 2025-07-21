#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# Activate the virtual environment
source "$HOME/src/open-drone-server/.env/bin/activate"

# Start your FastAPI backend
exec python main.py
