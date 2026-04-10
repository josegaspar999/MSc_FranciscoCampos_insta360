# Docker
This folder contains a Dockerfile that builds a ROS-based image with all necessary project dependencies baked in, as well as scripts to automate the image building and container creation process.
This allows for a reproducible and portable development environment that can be deployed on various platforms.
## Folder Structure and Contents
This folder contains the following files:
- Dockerfile: Image recipe with all necessary setup and dependencies
- vars.sh: Contains variables that are shared by the image building and the container creation scripts
- image_build.sh: Builds the Docker image from the Dockerfile
- container_create.sh: Creates a Docker container from the Docker image
## Building a Docker image
The image typically only needs to be built once. Subsequent builds are only necessary if changes are made to the Dockerfile.
The image can be built by calling `./image_build.sh`. The name of the image can be changed by changing `image_name = <desired_name>` in `vars.sh`.
## Creating a container
A container can be create using `./container_create.sh`. Multiple containers can be created from the same image, as long as they have different names. The name of the image can be changed by changing `container_name = <desired_name>` in `vars.sh`.
When building a container, it is important to ensure that the `shared_volume` in `vars.sh` points to the correct directory, containing the project code.
## Managing containers and images
A closed container can be opened with `docker start -i <container_name>`.
If a container is already running, a new terminal can be opened with `docker exec -it <container_name> bash`.
A container will automatically shutdown once its entrypoint process terminates. If that entrypoint is bash, exiting all terminals will automatically close the container.
It is also possible to forcefully close a running container from the outside with `docker stop <container_name>`.

To list all containers, use `docker ps -a`.
To list all images, use `docker images`.
A container can be deleted using `docker container rm <container_name>`.
An image can be deleted using `docker image rm <image_name>`.
