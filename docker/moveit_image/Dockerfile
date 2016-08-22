FROM osrf/ros:indigo-desktop-full

# Switch /bin/sh from dash to bash so that we can use `source`
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt-get update && \
    apt-get install -y python-wstool

RUN source /opt/ros/indigo/setup.bash && \
    mkdir moveit && \
    cd moveit && \
    mkdir src && \
    cd src/ && \
    wstool init . && \
    wstool merge https://raw.github.com/ros-planning/moveit_docs/indigo-devel/moveit.rosinstall && \
    wstool update && \
    cd .. && \
    rosdep install --from-paths src --ignore-src --rosdistro indigo -y && \
    catkin_make

RUN echo 'source /moveit/devel/setup.bash' >> ~/.bashrc
