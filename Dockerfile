FROM osrf/ros:humble-desktop

# Python and Warp
RUN apt-get update && apt-get upgrade -y
RUN apt install -y --no-install-recommends python3-pip
RUN pip install --upgrade setuptools
RUN pip install artefacts-client --extra-index-url https://d5cw4z7oemmfd.cloudfront.net/pep503/ -U

# Colcon, cmake, dependencies
RUN apt install python3-rosdep python3-colcon-common-extensions curl lsb-release git wget gnupg cmake build-essential -y

# Ignition Fortress
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt update -y && apt install ignition-fortress -y

WORKDIR /ws

# Simultor Environment is vital for colcon build to be succesful
ENV IGNITION_VERSION fortress


# Install test packages
COPY ./src/cam_test/package.xml /ws/src/cam_test/package.xml
RUN rosdep install --from-paths src --ignore-src -r -y
COPY ./src /ws/src
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install


# # Add a user with the same user_id as the user outside the container
# # Requires a docker build argument `user_id`
# ARG user_id
# ENV USERNAME developer
# RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
#  && echo "$USERNAME:$USERNAME" | chpasswd \
#  && adduser $USERNAME sudo \
#  && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

# Create a framebuffer as seen here: https://github.com/gazebosim/gz-rendering/blob/ign-rendering5/.github/ci/after_make.sh
RUN apt-get update -y \
  && apt-get -y install \
    xvfb \
  && rm -rf /var/lib/apt/lists/* /var/cache/apt/*
# RUN set -x
# RUN Xvfb :1 -ac -noreset -core -screen 0 1280x1024x24 &
# ENV DISPLAY=:1.0
# ENV RENDER_ENGINE_VALUES=ogre2
# ENV MESA_GL_VERSION_OVERRIDE=3.3

# # Commands below run as the developer user
# USER $USERNAME

# CMD ros2 run demo_nodes_cpp talker
COPY ./warp-client ./warp-client 
RUN pip install --editable ./warp-client
COPY ./warp.yaml .
CMD . /ws/install/setup.sh && xvfb-run warpcli run $ARTEFACTS_JOB_NAME
# CMD . /ws/install/setup.sh && xvfb-run ros2 launch cam_test sim1.launch.py output_video:=/ws/output/rec.mjpg