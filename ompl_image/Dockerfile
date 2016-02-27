FROM ubuntu:14.04

# Install add-apt-repository
RUN apt-get update && \
    apt-get install -y software-properties-common

# Necessary for libccd-dev (https://bitbucket.org/ompl/ompl/issues/79/ompl-on-ubuntu-1404)
RUN add-apt-repository ppa:libccd-debs/ppa

RUN apt-get update && \
    apt-get install -y build-essential && \
    apt-get install -y libboost-all-dev cmake libccd-dev python-dev \
    python-qt4-dev python-qt4-gl python-opengl freeglut3-dev libassimp-dev \
    libeigen3-dev libode-dev doxygen graphviz

# Install wget
RUN apt-get install -y wget

# According to http://ompl.kavrakilab.org/installation.html#install_linux
RUN wget https://bitbucket.org/ompl/ompl/downloads/omplapp-1.1.1-Source.tar.gz && \
    tar -zxvf omplapp-1.1.1-Source.tar.gz && \
    rm omplapp-1.1.1-Source.tar.gz && \
    cd omplapp-1.1.1-Source && \
    mkdir -p build/Release && \
    cd build/Release && \
    cmake ../..

# Install Python and GUI bindings
RUN cd omplapp-1.1.1-Source/build/Release && \
    make installpyplusplus && cmake . && \
    make update_bindings

# Run final make and make install
RUN cd omplapp-1.1.1-Source/build/Release && \
    make && \
    make test && \
    make install
