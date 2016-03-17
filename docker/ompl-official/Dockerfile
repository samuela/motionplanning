# Builds the official OMPL 1.1.1 from source.

FROM samuela/ompl-buildenv

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
    make installpyplusplus && \
    cmake . && \
    make update_bindings

# Run final make and make install
RUN cd omplapp-1.1.1-Source/build/Release && \
    make && \
    make test && \
    make install
