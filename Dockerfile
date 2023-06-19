FROM franklinselva/ros-noetic-desktop:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV SETUP_DIR=/home/ubuntu/drone-experiment/genom3-experiment/run

# Install for experiment
RUN mkdir -p $SETUP_DIR/
COPY ./install-scripts/setup-experiment.sh $SETUP_DIR/setup-experiment.sh
COPY ./install-scripts/drone-genom3.repos $SETUP_DIR/drone-genom3.repos
RUN bash $SETUP_DIR/setup-experiment.sh --robotpkg
RUN bash $SETUP_DIR/setup-experiment.sh --genom

# Add variables to bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN rm -rf $SETUP_DIR/*