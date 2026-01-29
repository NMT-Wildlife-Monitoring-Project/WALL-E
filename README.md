# WALL-E
Wildlife Activity Life Explorer
The main repository for the New Mexico Tech Wildlife Monitoring Project

# INSTALLATION

This system is designed to run in Docker on linux.  

Install docker  
<https://docs.docker.com/engine/install/ubuntu/>  

To use docker without sudo, run the following commands  
```
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

Ensure you have an SSH key set up with Github (optional)
<https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account>  

Clone this repository. To commit to the repository, clone with the ssh address. Don't push to main unless you helped write this README (make your own branch).
```
git clone https://github.com/NMT-Wildlife-Monitoring-Project/WALL-E.git
cd WALL-E
```

Install udev rules
```
cd scripts
sudo cp 99-walle-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Build the docker image  
This may take a while as several large packages are installed.  
```
cd docker
./start_docker.sh -b
```  

# USAGE

## Quick Start: From Zero to First Launch

### 1. Prerequisites
- Docker installed and user added to docker group (see INSTALLATION section above)
- Repository cloned: `cd WALL-E`
- udev rules installed (see INSTALLATION section above)

### 2. Build the Docker Image
```bash
cd docker
./start_docker.sh -b
```
This builds the Docker image with all ROS 2 packages, dependencies, and robot software.
**Note:** First build takes 10-20 minutes and requires internet access.

### 3. Launch Robot Bringup (Sensors & Motors)
Start all core robot processes (motors, LiDAR, IMU, GPS):
```bash
./start_docker.sh -r
```
This launches:
- RoboClaw motor driver
- LiDAR scanner
- IMU sensor
- GPS receiver
- Robot state publisher

### 4. Launch Navigation
Start GPS-based waypoint navigation with localization:
```bash
./start_docker.sh -n
```
This launches:
- Robot bringup (motors, sensors)
- GPS waypoint follower
- Nav2 navigation stack
- EKF localization
- RViz visualization (with X11 display forwarding)

### 5. Access the Container Shell
Open an interactive bash terminal to run custom commands:
```bash
./start_docker.sh
```

## Verification Checklist

After launching with `-r` or `-n`, verify the robot is functioning:

### List Active Topics
```bash
ros2 topic list
```

Expected topics:
- `/scan` - LiDAR scanner data
- `/odom` - Odometry from wheel encoders
- `/imu/data` - IMU sensor data
- `/fix` - GPS fix information
- `/tf` - Transform tree (frame relationships)

### Check TF Tree
```bash
ros2 run tf2_tools view_frames.py
```

Expected frames: `map` → `odom` → `base_link` → sensors

### Monitor Motor Status
```bash
ros2 topic echo /roboclaw_status
```

Should show motor speeds, encoder values, and battery voltage.

### Test Odometry
```bash
ros2 topic echo /odom
```

Should continuously update with robot position and velocity.

## Docker Commands Reference

### start_docker.sh Options
```
-r, --robot              Start robot bringup (sensors + motors)
-n, --navigate           Start navigation (GPS waypoint following)
-t, --teleop             Start joystick/teleop control
-l, --lidar              Start LiDAR sensor only
-c, --camera             Start USB camera
--imu                    Start IMU sensor
--gps                    Start GPS driver
-R, --rviz               Start RViz visualization (requires -d)
-w, --webapp             Start web control panel
-B, --rosbridge          Start rosbridge server
-d, --display            Enable X11 display forwarding (for RViz)
-b, --build              Build Docker image
-x, --stop               Stop running container
-i, --ros-domain-id <id> Set ROS domain ID (default: 62)
-h, --help               Show help message
```

### Common Usage Examples
```bash
# Build image
./start_docker.sh -b

# Start robot + RViz visualization
./start_docker.sh -r -R -d

# Start navigation + GPS
./start_docker.sh -n

# Start teleop control
./start_docker.sh -t

# Run custom command in container
./start_docker.sh -c "ros2 topic list"

# Interactive bash in container
./start_docker.sh
```

## Docker start script -- start_docker.sh
This script handles building and running the docker container, running commands inside the container, and setting up ros networking and environment.
```


# Cellular
<https://www.waveshare.com/wiki/SIM7600E-H_4G_HAT>

may need to reboot
'''
sudo nmcli dev disconnect wlan0
'''
ipv4 - trying wifi
'''
sudo systemctl restart NetworkManager
sudo systemctl restart zerotier-one.service
sudo zerotier-cli peers 
'''
should be ipv6

# 📡 Jetson Cellular Setup (SIM7600 USB Modem)

This project supports using a **SIM7600 USB LTE modem** on both Raspberry Pi 5 and NVIDIA Jetson platforms.
The connection uses **IPv6-only cellular** with **Cloudflare DNS64** to provide full IPv4 compatibility (GitHub, Docker, ROS, apt, etc.) even when the carrier does not provide IPv4.

* SIM7600 USB modem
* IPv6-only LTE
* Cloudflare DNS64/NAT64 workaround
* Using your saved `.nmconnection` file
* Fully portable for Pi → Jetson

These instructions assume the repo contains:

```
scripts/mint-cellular.nmconnection
```

which holds the working NetworkManager profile.

---

## 1. Install Required Networking Packages

Jetson devices often ship with a mixture of networking stacks.
Ensure NetworkManager + ModemManager are installed and active:

```bash
sudo apt update
sudo apt install -y network-manager modemmanager
```

Disable legacy networkd if it is enabled:

```bash
sudo systemctl disable systemd-networkd --now || true
sudo systemctl enable NetworkManager --now
```

Reboot:

```bash
sudo reboot
```

---

## 2. Plug In the SIM7600 Modem

After reboot, verify the modem is detected:

```bash
mmcli -L
```

Expected:

```
Found 1 modems:
    /org/freedesktop/ModemManager1/Modem/0 [SimCom] SIM7600
```

Confirm USB detection:

```bash
lsusb | grep -i sim
```

---

## 3. Install the NetworkManager Profile

From the project root:

```bash
cd WALL-E/scripts

sudo cp mint-cellular.nmconnection /etc/NetworkManager/system-connections/
sudo chown root:root /etc/NetworkManager/system-connections/mint-cellular.nmconnection
sudo chmod 600 /etc/NetworkManager/system-connections/mint-cellular.nmconnection
```

Reload NetworkManager:

```bash
sudo nmcli connection reload
```

---

## 4. Fix /etc/resolv.conf (Required!)

Some Jetson images ship with a static or immutable `/etc/resolv.conf`,
preventing NetworkManager from injecting the correct DNS64 servers.

Run:

```bash
sudo chattr -i /etc/resolv.conf 2>/dev/null || true
sudo rm -f /etc/resolv.conf
sudo ln -s /run/NetworkManager/resolv.conf /etc/resolv.conf
```

---

## 5. Connect to LTE

```bash
sudo nmcli connection up mint-cellular
```

The SIM7600 will automatically:

* Create an **IPv6-only data connection**
* Receive a global IPv6 address
* Use the baked-in **Cloudflare DNS64 servers**:

  ```
  2606:4700:4700::64
  2606:4700:4700::640
  ```
* Gain IPv4 compatibility via NAT64

---

## 6. Verify Connectivity

### IPv6 address

```bash
ip -6 addr show dev wwan0
```

Should show a global address like:

```
2607:xxxx:xxxx::xxxx/64
```

### IPv6 reachability

```bash
ping6 ipv6.google.com
```

### IPv4-only service through NAT64

```bash
git ls-remote https://github.com
docker pull ubuntu
curl -4 https://ifconfig.co
```

All of these should work even though the modem has **no IPv4 address**.

---

## 7. Auto-Connect on Boot (Optional)

```bash
sudo nmcli connection modify mint-cellular connection.autoconnect yes
```

---

## 8. Troubleshooting

### Check modem status

```bash
mmcli -m 0
```

### Check IPv6 routes

```bash
ip -6 route
```

### Check DNS64 is active

```bash
getent ahosts github.com
```

Should return a synthesized IPv6 address starting with:

```
64:ff9b::
```

If so, NAT64 is working.

---

# ✔ Summary

Once installed, the Jetson will:

* Use **IPv6-only LTE**
* Use **DNS64** to synthesize IPv4 addresses
* Access all IPv4-only services (GitHub SSH, Docker Hub, etc.)
* Maintain the exact same behavior as your Pi

Everything is driven by the included:

```
scripts/mint-cellular.nmconnection
```

so setup on a Jetson is as simple as **copy file → reload NM → connect**.

---

If you want, I can generate a **setup_cellular.sh** script that automates every step above, including safety checks, detection, and verification.
