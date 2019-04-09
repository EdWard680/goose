FROM ros:kinetic-ros-base

RUN apt update && apt install -q -y  \
	python3-dev python3-pip \
	build-essential \
	bluez bluez-tools \
	python-dev python-pip \
	libglib2.0-dev libboost-python-dev libboost-thread-dev libbluetooth-dev && \
	rm -rf /var/lib/apt/lists/*

RUN pip3 install pybluez
