FROM samuela/ompl

# Install pip and virtualenv
RUN apt-get update && \
    apt-get install -y python-pip && \
    pip install --upgrade pip && \
    pip install --upgrade virtualenv

RUN apt-get install -y python-matplotlib python-numpy

# Install miniconda
# RUN wget http://repo.continuum.io/miniconda/Miniconda-latest-Linux-x86_64.sh -O miniconda.sh && \
#     chmod +x miniconda.sh && \
#     ./miniconda.sh -b -p /miniconda && \
#     /miniconda/bin/conda update --yes conda && \
#     rm miniconda.sh
#
# ENV PATH /miniconda/bin:$PATH
#
# RUN conda install --yes matplotlib pyopengl
