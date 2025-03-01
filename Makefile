PROJECT ?= ros2-jazzy
VERSION ?= latest

USER ?= root
WORKSPACE ?= /home/root/ros2_ws
DOCKER_IMAGE ?= $(PROJECT):$(VERSION)

SHMSIZE ?= 444G
DOCKER_OPTS := \
				--name $(PROJECT) \
				--rm -it \
				--shm-size $(SHMSIZE) \
				-e DISPLAY=$(DISPLAY) \
				-e XAUTHORITY \
				-e ~/.cache:/home/root/.cache \
				-v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 \
				-v /var/run/docker.sock:/var/run/docker.sock \
				-v ${PWD}/src:${WORKSPACE}/src \
				-w ${WORKSPACE} \
				--privileged \
				--ipc=host \
				--network=host \
				--device /dev/ttyACM0:/dev/ttyACM0

docker-build:
	docker build \
			-f docker/Dockerfile \
			-t $(DOCKER_IMAGE) .

docker-run:
	docker-build
	xhost +local: && docker run $(DOCKER_OPTS) $(DOCKER_IMAGE) /bin/bash -c "${COMMAND}"

docker-interactive:
	xhost +local: && docker run $(DOCKER_OPTS) $(DOCKER_IMAGE) /bin/bash