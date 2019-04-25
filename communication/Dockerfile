FROM ros:kinetic-ros-base-xenial

RUN apt update && \
    apt upgrade -y

RUN apt install -y python-pip python3-pip

RUN apt install -y \
    ros-kinetic-mqtt-bridge

RUN rm -rf /var/lib/apt/lists/

RUN pip install \
    inject \
    paho-mqtt \
    msgpack-python

RUN pip3 install pyyaml

RUN mkdir communication
WORKDIR communication

COPY . ./

CMD ["/bin/bash", "start.sh"]
