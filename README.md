[Get Started](https://github.com/snelsondurrant/coug_fgo_dev?tab=readme-ov-file#get-started)

--

### Get Started

> **NOTE:** Newer MacBooks with ARM64 CPU architecture (M1, M2, etc) have not been extensively tested.

- Install Docker and set up the Linux development environment.

  **Windows:**

  - Install WSL2 on your Windows machine by following the instructions [here](https://docs.microsoft.com/en-us/windows/wsl/install).

  - Install Docker Desktop on your Windows machine by following the instructions [here](https://docs.docker.com/desktop/), and enable the WSL2 backend by following the instructions [here](https://docs.docker.com/desktop/windows/wsl/).

  **Linux:**

  - Install Docker Engine on your Linux machine by following the instructions [here](https://docs.docker.com/engine/install/ubuntu/). Make sure to follow the post-installation steps to allow your user to run Docker commands without `sudo`.

- Open a new terminal and clone the `coug_fgo_dev` repository.

  ```bash
  git clone https://github.com/snelsondurrant/coug_fgo_dev.git
  ```

- Enter the repository and run `bash compose.sh` to build and launch the Docker container.

  ```bash
  cd coug_fgo_dev && bash compose.sh
  ```

- Exit the container and follow the instructions [here](https://github.com/byu-holoocean/holoocean-ros/tree/main/docker/runtime) to build a runtime Docker image for `holoocean-ros`. When prompted to run `./build_container.sh`, specify the branch `nelson/fgo-dev` using `./build_container.sh -b nelson/fgo-dev`.

- When finished, launch HoloOcean outside of the container using `bash holoocean_launch.sh`.

- Open a new terminal, enter the container using `bash compose.sh`, build the `coug_ws` workspace, and launch the localization nodes and development tools using `bash dev_launch.sh`.

  ```bash
  cd ~/coug_ws && colcon build
  cd ~/scripts && bash dev_launch.sh
  ```

--

Created by Nelson Durrant, Nov 2025.
