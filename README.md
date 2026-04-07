[![Docker image (amd64)](https://github.com/MrDuartePT/ros2-rob-trsa-template/actions/workflows/publish-docker-amd64.yml/badge.svg)](https://github.com/MrDuartePT/ros2-rob-trsa-template/actions/workflows/publish-docker-amd64.yml)
[![Docker image (arm64)](https://github.com/MrDuartePT/ros2-rob-trsa-template/actions/workflows/publish-docker-arm64.yml/badge.svg)](https://github.com/MrDuartePT/ros2-rob-trsa-template/actions/workflows/publish-docker-arm64.yml)
[![Docker Image Size](https://badgen.net/docker/size/mrduartept/ros2-rob-trsa-template/jazzy?icon=docker&label=image%20size)](https://hub.docker.com/r/mrduartept/ros2-rob-trsa-template/)
[![Docker Pulls](https://badgen.net/docker/pulls/mrduartept/ros2-rob-trsa-template?icon=docker&label=pulls)](https://hub.docker.com/r/mrduartept/ros2-rob-trsa-template/)

# ros2-rob-trsa-template

Template for ROS2 workspace using [VS Code Dev Containers](https://code.visualstudio.com/docs/remote/containers) & [Docker Compose](https://docs.docker.com/compose/).

- Use `Dev Containers: Clone Repository in Container Volume...` in VS Code's Command Palette to auto-magically setup everything!
  - To re-open repository, either use `File > Open Recent` or repeat the above with the **exact SAME url**.
- For customization, do a global find & replace for `(OPTION)`.
  - Remember to `Dev Containers: Rebuild Container` afterwards!
- GUI apps are viewable via [noVNC](https://novnc.com/info.html) (VNC client web app) hosted on <http://localhost:6080/>.
- By default the docker-compose.yaml will use images publish to Github/Docker Hub (same version of master)
  - https://hub.docker.com/r/mrduartept/ros2-rob-trsa-template
  - https://github.com/mrduartept/ros2-rob-trsa-template/pkgs/container/ros2-rob-trsa-template

## Distrobox users 
This image was confirmed working by a user.

If you prefer to use it that way, just run the commands below.

Note: --root is added to the command, but depending on your usage, it might not be needed.

For NVIDIA Users:
```bash
# Change the second --volume for your ros distro folders
distrobox create --root -i mrduartept/ros2-rob-trsa-template:humble --yes --nvidia --volume /usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d:r --volume $HOME/Documents/ros-workspaces:$HOME/Documents/ros-workspaces:rw
```
For Intel/AMD Users
```bash
# Change the second --volume for your ros distro folders
distrobox create --root -i mrduartept/ros2-rob-trsa-template:humble --yes  --volume /usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d:r --volume $HOME/Documents/ros-workspaces:$HOME/Documents/ros-workspaces:rw
```

## Table of Contents

- [Mount Point `./rob_ws` and `./trsa_ws`](#mount-point-data)
- [Other Features](#other-features)
  - [`requirements.txt` Escape Hatch](#requirementstxt-escape-hatch)
  - [VS Code Tasks](#vs-code-tasks)
  - [VS Code Extension Suggestions](#vs-code-extension-suggestions)
  - [Code Formatting](#code-formatting)
  - [Dev Container using Docker Compose](#dev-container-using-docker-compose)
- [Tips](#tips)
  - [Update Package Indexes](#update-package-indexes)
  - [Change ROS Distro](#change-ros-distro)
  - [Docker Image Distribution](#docker-image-distribution)
- [Troubleshooting](#troubleshooting)


## Mount Point `./rob_ws` and `./trsa_ws`

Mount points are used to mount a folder from the host into the container. 
The `./rob_ws` and `./trsa_ws` are two seperates workspace, one for Robotics and other for Telerobotics

`./rob_ws` and `./trsa_ws` can be used to share anything from the host with the container, is also the place on were you create your ros packages
Also remember to:
- Add mount points to [`.gitignore`](./.gitignore) and [`.dockerignore`](./.dockerignore).

## Other Features

### `requirements.txt` Escape Hatch

Some dependencies may be unavailable from the `rosdep` package manager (check [ROS Index](https://index.ros.org)). For Python dependencies, they should be added to a `requirements.txt` created within the ROS package. The ROS package's `requirements.txt` should then be composed into the workspace's [`requirements.txt`](./requirements.txt) (example in [`requirements.txt`](./requirements.txt)). For other dependencies, they should be added to both Dockerfiles. See <https://github.com/ros/rosdistro/blob/master/CONTRIBUTING.md#rosdep-rules-contributions> with regards to adding new packages to `rosdep`.

### Alias
There are two alias that can be used to create ros packages:
- `ros2_cmake_pkg`: Create a Ros2 C/C++ package in the current directory
- `ros2_python_pkg`: Create a Ros2 Python package in the current directory

There also alias to auto-load the Isaac Sim and Issac Lab python envrioment, called `isaacload`
This enviroment is only created `isaac_ros-dev` folder is mounted in the container, look at [`postCreate.sh`](./.devcontainer/hooks/postCreate.sh) and [`issaclab-pyenv.sh`](./.devcontainer/scripts/issaclab-pyenv.sh)

### VS Code Tasks

For developer convenience, some common tasks are present in [`tasks.json`](./.vscode/tasks.json). Use them by opening the Command Palette and typing `task`. The following tasks are available:

- `rosdep install dependencies`: Install all dependencies of the current workspace.
- `colcon build all`: Build all (detected) ROS packages.
- `colcon build specific`: Build a specific ROS package.
- `update package index`: Update Ubuntu and ROS package indexes.

### VS Code Extension Suggestions

Both [`devcontainer.json`](./.devcontainer/devcontainer.json) and [`extensions.json`](./.vscode/extensions.json) are configured such that VS Code will automatically install some extensions. These are highly recommended for developer experience.

### Code Formatting

[`black`](https://github.com/psf/black) is used to format Python code due to its uncompromising design. Imports are also automatically sorted using compatible rules. See [`settings.json`](./.vscode/settings.json) for the configuration.

### Dev Container using Docker Compose

Instead of using Dev Container with a Dockerfile, this template uses it with Docker Compose instead. This is done for a few reasons:

- Include example of how to configure Docker for ROS to communicate across containers.
- Docker Compose is closer to real world deployment scenarios.
  - It is also more flexible and easier to configure than [`devcontainer.json`](./.devcontainer/devcontainer.json).

## Tips

### Update Package Indexes

The `rosdep` and Ubuntu package managers rely on a local cache of their package index. If the package index is outdated, it may not contain any active package distribution server, leading to package downloads failing. Hence, it is recommended to periodically re-download the package index:

```sh
apt-get update
rosdep update
```

### Change ROS Distro

To change ROS Distro, do a global search for the current distro (`jazzy`) and replace as necessary with the new distro. Afterwards, rebuild the Dev Container.

### Docker Image Distribution

See <https://docs.docker.com/engine/reference/commandline/docker/> for more info.

#### Building

> Note: If using `Dev Containers: Clone Repository in Container Volume...` on Windows, follow <https://code.visualstudio.com/docs/containers/choosing-dev-environment#_windows-subsystem-for-linux> to ensure built images are stored on the host computer's image repository.

```sh
docker build . -t ros-jazzy-rob-trsa:vx.x.x -t ros-jazzy-rob-trsa:latest
```

Images can have multiple names tagged to them. Tagging images with the version number and as latest helps when distributing images.

#### Exporting

> Note: Run this command on the host computer rather than in the Dev Container.

```sh
docker save ros-jazzy-rob-trsa:vx.x.x ros-jazzy-rob-trsa:latest -o ros-jazzy-rob-trsa-vx.x.x.tar
```

Compressing the image afterwards using `xzip` is recommended to save space. Docker is able to load compressed images (i.e., `ros-jazzy-rob-trsa-vx.x.x.tar.xz`) without decompressing manually.

#### Importing

```sh
docker load -i ros-jazzy-rob-trsa-vx.x.x.tar.xz
```

Imports the image and its names. It will still be tagged as `ros-jazzy-rob-trsa:vx.x.x` and `ros-jazzy-rob-trsa:latest`, conveniently replacing the previous `latest`.

## Troubleshooting

### Setup Issues

- On first setup, the Dev Container fails to build and complains about `/r`.
  - The Dev Container lifecycle hooks were checked out with CRLF instead of LF line endings.
  - Do `git config --global core.autocrlf input` and re-clone the repository.
  - See <https://code.visualstudio.com/docs/remote/troubleshooting#_resolving-git-line-ending-issues-in-wsl-resulting-in-many-modified-files> for more info.
- When modifying the Dockerfile, earlier cached steps might be stale, causing later steps to fail.
  - Rebuild the Dev Container without cache.

### Build Issues

- Try deleting the `build` and `install` folders before rebuilding all packages.

### Runtime Issues

- `rosdep` has no version lock, which means there is no protection against breaking changes when packages update.
  - See: <https://github.com/ros-infrastructure/rosdep/issues/325>
  - One solution would be to use your own install script instead of `rosdep`.
- ROS `launch` files aren't symlinked unlike Python code.
  - Rebuild the package when `launch` files are modified.

# Documentation
https://docs.ros.org/en/jazzy/Installation.html

https://docs.ros.org/en/jazzy/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html

https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/index.html

https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html