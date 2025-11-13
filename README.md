[Get Started](https://github.com/snelsondurrant/coug_fgo_dev?tab=readme-ov-file#get-started)

--

### Get Started

**Windows:**

- Install WSL2 on your Windows machine by following the instructions [here](https://docs.microsoft.com/en-us/windows/wsl/install).

- Install Docker Desktop on your Windows machine by following the instructions [here](https://docs.docker.com/desktop/), and enable the WSL2 backend by following the instructions [here](https://docs.docker.com/desktop/windows/wsl/).

- Open a new WSL2 terminal and clone the coug_fgo_dev repo into your WSL2 environment using `git clone https://github.com/snelsondurrant/coug_fgo_dev.git`.

- Run `cd coug_fgo_dev && bash compose.sh` to build the Docker image and launch the container.

- Exit the container and follow the README [here](https://github.com/byu-holoocean/holoocean-ros/tree/main/docker/runtime) to set up Holoocean-ROS in your WSL2 environment. When prompted to run `./build_container.sh`, specify the branch 'nelson/fgo-dev' using `./build_container.sh -b nelson/fgo-dev`.

- When finished, launch HoloOcean outside of the container using `bash holoocean_launch.sh`. Open a new WSL2 terminal, enter the container using `bash compose.sh`, and run `cd ~/coug_ws && colcon build` followed by `cd ~/scripts && bash dev_launch.sh` to launch the localization nodes and development tools.

**Linux:**

- Install Docker Engine on your Linux machine by following the instructions [here](https://docs.docker.com/engine/install/ubuntu/). Make sure to follow the post-installation steps to allow your user to run Docker commands without `sudo`.

- Open a new terminal and clone the coug_fgo_dev repo into your Linux environment using `git clone https://github.com/snelsondurrant/coug_fgo_dev.git`.

- Run `cd coug_fgo_dev && bash compose.sh` to build the Docker image and launch the container.

- Exit the container and follow the README [here](https://github.com/byu-holoocean/holoocean-ros/tree/main/docker/runtime) to set up Holoocean-ROS. When prompted to run `./build_container.sh`, specify the branch 'nelson/fgo-dev' using `./build_container.sh -b nelson/fgo-dev`.

- When finished, launch HoloOcean outside of the container using `bash holoocean_launch.sh`. Open a new WSL2 terminal, enter the container using `bash compose.sh`, and run `cd ~/coug_ws && colcon build` followed by `cd ~/scripts && bash dev_launch.sh` to launch the localization nodes and development tools.

--

Created by Nelson Durrant, Nov 2025.