# MRSD Team G: ORIO
## Desktop Setup

### Install Prerequisites:
- **Setup Docker:** 
   - [Install Docker](https://docs.docker.com/engine/install/ubuntu/)
   - [Install Nvidia Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
   - (Optional) Add docker to [sudoers](https://docs.docker.com/engine/install/linux-postinstall)\
      Do not forget to logout and login after adding docker to sudoers\
      If not done, please run all docker commands with a `sudo` prefix.

- **Setup Frankapy:**\
   Follow instructions from the official [frankapy](https://github.com/iamlab-cmu/frankapy) repository and clone the frankapy repository at `/home/student`


### Setup the PC:
1. Clone the [orio-main](https://github.com/TeamG-ORIO/orio-main) repository
   ```bash
   git clone git@github.com:TeamG-ORIO/orio-main.git
   ```

2. Build the Docker Container:
   ```
   docker build -t orio_dev_env -f Dockerfile_orio .
   ```

### Running Demo Code:
**Note:** Perform this process for both arms
**Note:** Replace `[control-pc-name]` with the name of the control pc, for example, `iam-snowwhite`
1. Unlock robot joints
   ```bash
   ssh -X student@[control-pc-name]
   google-chrome
   ```
   In google chrome open `https://172.16.0.2/desk/` and press `Click to unlock joints`

2. Run `roscore` on the control pc. In a new terminal:
   ```bash
   roscore
   ```

3. Run the start_control_pc script from the frankapy package
   ```bash
   cd <frankapy package directory>
   bash ./bash_scripts/start_control_pc.sh -u student -i [control-pc-name]
   ```
   This should launch 3 terminals which sets up frankapy to communicate with the robot. To reset frankapy communication, just kill the 3 terminals and rerun the script.
   
4. **Running the Docker Container:**
   - Run the Docker Container. In a new terminal:
      ```bash
      bash orio_run_docker.sh
      ```

   - Attach a terminal connected to the Docker Container:
      ```bash
      bash orio_terminal_docker.sh
      ```
      
5. **Running the System:**
   - Run each of the following in a separate Docker terminal:
      ```bash
      roslaunch manipulation cameras.launch
      ```
      ```bash
      cd src/devel_packages/orio
      python3 state_machine_encore.py
      ```
      ```bash
      cd src/devel_packages/orio/rfid
      python3 db_manager.py
      ```
      ```bash
      cd src/devel_packages/orio/rfid
      python3 inventory_dashboard.py
      ```
   

