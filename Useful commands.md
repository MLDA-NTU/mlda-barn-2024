# Docker

## Build from Dockerfile

```shell
docker build . -t barn:icra --no-cache

sudo docker run --rm -dt --name barn \
	--gpus all \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	barn:april1

docker push mldarobotics/barn2024

```

## Tag and push to DockerHub

```
docker tag barn:april1 mldarobotics/barn2024:april1
docker push mldarobotics/barn2024:april1
```

## Pull from the DockerHub

```shell
docker run --rm -dt --name barn \
	--gpus all \
	-e DISPLAY="$DISPLAY" \
	-e QT_X11_NO_MITSHM=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v ~/mlda-barn-2024:/jackal_ws/src/mlda-barn-2024 \
	mldarobotics/barn2024:april1


docker run --rm -dt --name barn \
	--gpus all \
	-e DISPLAY="$DISPLAY" \
	-e QT_X11_NO_MITSHM=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	mldarobotics/barn2024:april1

docker run --rm -dt --name barn \
	-e DISPLAY=":1" \
	-e QT_X11_NO_MITSHM=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	barn:april1

```

## ROS

- Clear map

```shell
rosservice call /move_base/clear_costmaps "{}"
```
