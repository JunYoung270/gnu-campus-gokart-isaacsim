#!/bin/bash
set -e

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Go to project root
cd "$SCRIPT_DIR/.."

# Check if nvidia-smi is available
if ! command -v nvidia-smi &> /dev/null; then
    echo "Error: nvidia-smi not found. Please ensure NVIDIA drivers are installed."
    exit 1
fi

# Prepare X11 access
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi
chmod 777 $XAUTH

# Detect optional MOZA R5 devices
MOZA_ARGS=()

MOZA_EVENT_LINK=$(compgen -G "/dev/input/by-id/usb-Gudsen_MOZA_R5_Base_*-if02-event-joystick" | head -n1 || true)
MOZA_JS_LINK=$(compgen -G "/dev/input/by-id/usb-Gudsen_MOZA_R5_Base_*-if02-joystick" | head -n1 || true)

if [ -n "$MOZA_EVENT_LINK" ] && [ -n "$MOZA_JS_LINK" ]; then
    MOZA_EVENT=$(readlink -f "$MOZA_EVENT_LINK")
    MOZA_JS=$(readlink -f "$MOZA_JS_LINK")

    echo "MOZA R5 detected:"
    echo "  event: $MOZA_EVENT"
    echo "  joystick: $MOZA_JS"

    MOZA_ARGS+=(--device="$MOZA_EVENT:/dev/input/moza-event:rwm")
    MOZA_ARGS+=(--device="$MOZA_JS:/dev/input/moza-js:r")
else
    echo "Warning: MOZA R5 not detected. Starting without MOZA devices."
fi

# Run the container
echo "Running Docker container..."
docker run --rm -it \
    --device nvidia.com/gpu=all \
    --device /dev/dri \
    "${MOZA_ARGS[@]}" \
    --network host \
    --workdir /workspace/autoware_off-road_sim \
    -e "DISPLAY=${DISPLAY}" \
    -e "XAUTHORITY=${XAUTH}" \
    -e "NVIDIA_DRIVER_CAPABILITIES=all" \
    -e "NVIDIA_VISIBLE_DEVICES=all" \
    -v $XAUTH:$XAUTH \
    -v $(pwd)/docker/entrypoint.sh:/usr/local/bin/entrypoint.sh \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/workspace/autoware_off-road_sim \
    --name autoware_off-road_sim \
    autoware_off-road_sim:6.0 \
    "$@"
