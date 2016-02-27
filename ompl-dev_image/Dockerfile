FROM samuela/ompl

# Install pip and virtualenv
RUN apt-get update && \
    apt-get install -y python-pip && \
    pip install --upgrade pip && \
    pip install --upgrade virtualenv

RUN apt-get install -y python-matplotlib python-numpy
