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

## Docker start script -- start_docker.sh
This script handles building and running the docker container, running commands inside the container, and setting up ros networking and environment.
WARNING: The following options are not implemented yet.
```
--start (-s)
--teleop (-t)
--usb-cam (-u)
--video-stream (-v)
--mapping (-M)
--view-map (-w)
--motors (-g)
--rosbridge (-B)
```
```
Usage: ./start_docker.sh [ --start (-s) | --teleop (-t) | --usb-cam (-u)     | --video-stream (-v) | --mapping (-M) | --view-map (-w)     | --motors (-g) | --rosbridge (-B) --command (-c) <command>]      [ --ros-domain-id (-i) <id> | --copy (-C) <from> <to> | --display (-d)     | --build (-b) | --stop (-x) | --restart (-R) | --quiet (-q) | --help (-h) ]
This script is used to start and manage a Docker container for WALL-E the wildlife monitoring robot.
If no IP addresses are specified, the script will attempt to determine them from the hostname. If this fails, try setting the hostname or IP.
If no action is specified, the script will open an interactive bash terminal in the container.
Actions (pick ONE):
  --start (-s)                Start all processes on the robot
  --teleop (-t)               Run joystick control
  --usb-cam (-u)              Run usb camera node
  --video-stream (-v)         View the video stream
  --mapping (-M)              Run mapping
  --view-map (-w)             Run map view
  --motors (-m)               Run motor control
  --rosbridge (-B)            Run rosbridge server
  --command (-c) <command>    Pass a command to be run in the container
Options:
  --ros-domain-id (-i) <id>   Set the ROS domain ID (default: 62)
  --copy (-C) <from> <to>     Copy files from the container to the host
  --display (-d)              Enable display support (forward X11 display)
  --build (-b)                Build the Docker container (will stop the running container if any)
  --stop (-x)                 Stop the running Docker container
  --restart (-R)              Restart the Docker container if it is running
  --quiet (-q)                Suppress output
  --help (-h)                 Show this help message
  ```

# Ros Workspace
TODO: Basically all of these

## ros2_roboclaw_driver
TODO: Configure this
Separate ros workspace

## robot_bringup

## robot_description

## robot_motors
we might not need this one

## robot_navigation

## robot_teleop
Run the teleop node
'ros2 launch teleop teleop_launch.py'
See <https://wiki.ros.org/joy> and <https://wiki.ros.org/teleop_twist_joy>. The documentation is for ros 1 but the parameters are the same for the most part. Use
`ros2 param list` when running the node to see available parameters.

## robot_web_interface

## sllidar_ros2
This package is for rplidar laserscan sensors.

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
