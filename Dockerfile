# Replace all instances of shujingke with your github / docker.io / system username.
# Replace hk.archive.ubuntu.com with your own country code, e.g. nl.archive.ubuntu.com
# adduser shujingke
# cd ~ && git clone http://shujingke@github.com/shujingke/opencog && cd opencog && git pull
# docker build -t shujingke/opencog-dev-qt .
# xhost +
# docker run -t shujingke/opencog-dev-qt -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /home/shujingke:/home/shujingke 

# New run command:
# docker run --rm -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -v /home/shujingke:/home/shujingke -e DISPLAY=:0.0 -p 17001:17001 -t shujingke/opencog-dev-qt

# Newer run command:
# ssh -X hostname 'docker run --rm -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -v /home/shujingke:/home/shujingke -e DISPLAY=$DISPLAY -p 17001:17001 -t shujingke/opencog-dev-qt'

FROM ubuntu:14.04
MAINTAINER Alex van der Peet "alex.van.der.peet@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"

RUN sed 's/archive.ubuntu.com/hk.archive.ubuntu.com/' -i /etc/apt/sources.list

RUN apt-get -y update
RUN apt-get -y install software-properties-common wget tmux qtcreator \
                       lxterminal git gitk git-gui meld lxde

ADD scripts/ocpkg install-dependencies-trusty
RUN chmod +x /install-dependencies-trusty
RUN /install-dependencies-trusty

ENV DISPLAY 0:0

ENV STARTSCRIPT "\
echo evaluating startup script... &&\
tmux new-session -d 'echo -e \"\e[1;34mbash\e[0m\" ; /bin/bash' &&\
tmux set -g set-remain-on-exit on &&\
tmux set-option -g set-remain-on-exit on &&\
tmux bind-key R respawn-window &&\
tmux split-window -d -v -p 25 'echo -e \"\e[1;34mqtcreator\e[0m\" ;\
  /usr/bin/qtcreator' &&\
tmux split-window -d -v -p 25 'echo -e \"\e[1;34mqtcreator\e[0m\" ;\
  /usr/bin/lxterminal' &&\
tmux select-layout even-vertical &&\
tmux attach \
"

CMD /bin/bash -l -c "eval $STARTSCRIPT"
