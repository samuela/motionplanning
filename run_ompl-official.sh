#!/usr/bin/env bash

# http://kartoza.com/how-to-run-a-linux-gui-application-on-osx-using-docker/

# http://stackoverflow.com/questions/1908610/how-to-get-pid-of-background-process
socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\" &
SOCAT_PID=$!

# http://stackoverflow.com/questions/13322485/how-to-i-get-the-primary-ip-address-of-the-local-machine-on-linux-and-os-x
LOCAL_IP=$(ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1' | head -n 1)

# -t                        Allocate a pseudo-TTY
# -i                        Keep STDIN open even if not attached
# --rm                      Automatically remove the container when it exits
# -p 8888:8888              Reveal port 8888 (for Jupyter notebook)
# -e DISPLAY=$LOCAL_IP:0    Environment variable for X11 forwarding
docker run \
  -t \
  -i \
  --rm \
  -p 8888:8888 \
  -e DISPLAY=$LOCAL_IP:0 \
  -v ~/Research/motionplanning/:/motionplanning \
  -v ~/Research/pyDPMP/:/pyDPMP \
  samuela/ompl-official bash -c "pip install -e /pyDPMP && bash"

kill $SOCAT_PID
