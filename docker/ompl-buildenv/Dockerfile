# The base Docker image for all OMPL Docker things. Includes all of the
# necessary dependencies.

FROM ubuntu:14.04

# Install add-apt-repository. Necessary for libccd-dev.
# See https://bitbucket.org/ompl/ompl/issues/79/ompl-on-ubuntu-1404.
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository ppa:libccd-debs/ppa

RUN apt-get update && \
    apt-get install -y build-essential wget && \
    apt-get install -y libboost-all-dev cmake libccd-dev python-dev \
    python-qt4-dev python-qt4-gl python-opengl freeglut3-dev libassimp-dev \
    libeigen3-dev libode-dev doxygen graphviz
