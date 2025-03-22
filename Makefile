PROJECT ?= ros2-jazzy
VERSION ?= latest

USER ?= root
WORKSPACE ?= /home/root/ros2_ws
ROOT ?= /home/root
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
				-v ${PWD}/main_ws:${WORKSPACE} \
				-v ${PWD}/uros_ws:${ROOT}/uros_ws \
				-w ${WORKSPACE} \
				--privileged \
				--ipc=host \
				--network=host \
				--device /dev:/dev

docker-build:
	docker build \
			-f docker/Dockerfile \
			-t $(DOCKER_IMAGE) .

docker-run:
	docker-build
	xhost +local: && docker run $(DOCKER_OPTS) $(DOCKER_IMAGE) /bin/bash -c "${COMMAND}"

docker-interactive:
	xhost +local: && docker run $(DOCKER_OPTS) $(DOCKER_IMAGE) /bin/bash