#! /usr/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Building docker image..."
echo "==========================================="
cd ${SCRIPT_DIR} && docker compose -f docker/docker-compose.yml build

xhost +local:docker

echo "Starting robot chaser..."
echo "==========================================="
cd ${SCRIPT_DIR} && docker compose -f docker/docker-compose.yml run --remove-orphans barista-dev
