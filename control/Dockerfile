FROM duckietown/rpi-duckiebot-base

RUN pip install filterpy numpy

WORKDIR catkin_ws/src
RUN catkin_create_pkg goose_control

RUN mkdir goose_control/src
COPY . ./goose_control/src

#RUN catkin_make goose_control

CMD ["/bin/bash", "goose_control/src/start.sh"]
