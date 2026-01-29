# NURC UROV

### Setup
For this project, we will be leveraging ROS, the Robot Operating System. Setting up the ROS development environment can be a little bit of a time commitment, so be prepared to spend some time working on this.
The setup procedure depends on your OS.
##### Linux (Ubuntu 24.04 LTS ONLY)
If, for some reason, you are a Linux user on the latest Ubuntu LTS, congratulations, you probably have the easiest setup. The below is the instructions straight from OSRF (the people behind ROS)
1. Check Locale
Run the following command. As long as it says something with UTF-8 in it, you are good.
```bash
locale
```
2. Enable required repositories
Run the following comands
```bash
sudo apt install software-properties-common

sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
```
3. Install development tools
```bash
sudo apt update && sudo apt install ros-dev-tools
```
4. Install ROS 2
```bash
sudo apt update && sudo apt upgrade

sudo apt install ros-jazzy-desktop

# note that if you use a different shell (probably ZSH, replace setup.bash with setup.zsh and .bashrc with .zshrc)
echo "source /opt/ros/jazzy/setup.bash" > .bashrc && source .bashrc
```

5. Make sure everything works

Run the following in a terminal
```bash
ros2 run demo_nodes_cpp talker
```

Open another terminal and run the command below
```bash
ros2 run demo_nodes_py listener
```

You should see the ROS 2 talker and listener working. 

#### Linux (Not Ubuntu 24.04 LTS)
The ROS 2 binaries that we use are only built for Ubuntu LTS. DO NOT TRY TO BUILD THEM FROM SOURCE. Instead we will just use something called distrobox to run the instance.

1. Install distrobox

```bash
# install with your package manager
sudo apt install distrobox # Debian, Ubuntu, POP_OS, Mint, etc.
sudo dnf install distrobox # Fedora, Bazzite, etc.
sudo zypper install distrobox # OpenSUSE
sudo pacman -S distrobox # Arch, CachyOS, EndeavourOS, etc.
```
2. Create your distrobox container
This will create a distrobox container running Ubuntu Noble Numbat, which is 24.04 LTS
```bash
distrobox create -n ROS -i ubuntu:noble
```

3. Enter your container
```bash
distrobox enter ROS
```

4. Follow the instructions for Ubuntu 24.04 LTS + additional notes

Note that you will have to enter your distrobox container every time you want to work with ROS. You might also want to be able to run commands on your host system while in your container (i.e. things like spwaning VSCode, Neovim, etc.) To run a command on your host system while in a distrobox continer, prepend whatever command you will run with `distrobox-host-exec`. Examples below
```bash
distrobox-host-exec code . # open VSCode
distrobox-host-exec nvim . # open Neovim
```
You also don't want to run this line either
```bash
echo "source /opt/ros/jazzy/setup.bash" > .bashrc && source .bashrc
```
Instead, go into your shell's configuration file (`~/.bashrc`, `~/.zshrc`, etc) and add this function
```bash
ros-init() {
    source /opt/ros/jazzy/setup.bash # replace .bash with your shell if you are running a different shell (zsh)
}
```
And then source it
```bash
source .bashrc # or .zshrc, etc.
```

#### Windows
ROS 2 technically has binaries for Windows 10 (not 11), but don't bother with them. Instead, you will have to run a Linux environment somehow.
You have two options: dual booting or running a virtual machine. Dual booting will get you extra performance as you are running on bare metal, but it comes with the pains of running two operating systems (one of which being Windows, which sometimes doesn't like this) on your computer. Virtual machines will add overhead and can only use a portion of your computer's resources, reducing performance, but are simpler to set up.

##### Windows. - dual boot
Ideally, you would install Ubuntu 24.04 LTS and then just follow the instructions for Ubuntu 24.04, but this isn't a great idea for newer hardware (Ubuntu ships older kernels, which includes device drivers, making running Ubuntu a real pain in the ass on some laptops). You could dual boot another Linux distro, but then you will have to run in a distrobox container.

If you want to dual boot, talk to whoever the UROV software lead is and they should be able to help you. If you have to do this on your own, follow the rough outline below.

1. Pick a distro

Again, pick a Linux distro. Ubuntu 24.04 LTS is ideal, but some new hardware might prefer something like Fedora. Download an ISO file (this is a disk image) for the distro. You will want to install the amd64/X86-64 version (unless your computer uses a Qualcomm CPU, at which point you should just use a virtual machine instead)

2. Flash a boot USB drive

Get your hands on a USB flash drive that you are comfortable with completely wiping. Install Rufus on Windows, and use it to turn the USB drive into a boot drive.

3. Install Linux

With the boot USB still in your computer, reboot. While you are rebooting, spam the key that will take you to your laptop's BIOS. In your BIOS, turn off secure boot. You should also be able to go into a boot menu and boot into your USB. You should see that you will boot into a desktop environment. Follow the instructions given by the installer and you should be good.

4. Follow Linux setup guide

When you are done with the dual boot process. Follow the setup guide for your Linux distro. Note that to boot back into Windows / Linux, you will have to reboot your system every time.


##### Windows - virtual machine
1. Install VM software

You can pick any virtual machine software for this. Common examples are Virtual Box and VMWare. We recommend VMWare, if you can manage to navigate the website's UI.

2. Install Ubuntu 24.04 ISO

Go to the Ubuntu website and install the ISO for Ubuntu 24.04. You'd want the amd64 version unless you are using a Qualcomm CPU at which point you'd want the arm64/aarch64 version.

3. Set up a virtual machine

Open your virtual machine software and follow the instructions to create a new Ubuntu 24.04 virtual machine. Follow the installer instructions and once you are all installed, follow the Ubuntu 24.04 setup steps.

#### MacOS
MacOS gives you one option - virtual machines. Follow the Windows - virtual machine instructions to set up ROS 2. The virtual machine software we recommend for Mac is VMWare Fusion (be aware that navigating the website to actually install this is way more complicated than it needs to be), but you can also use UTM which is way easier to install, but might yield worse performance or Parallels, which while technically the best, costs money. Note that if you are using Apple silicon (M1, M2, M3, etc.) you will need to use the arm64/aarch64 ISO for Ubuntu 24.04.

### Temporary roadmap:
1. Establish communication between laptop on surface and raspberry pi
2. Create thruster node - subscribe to movement commands, calculate individual motor speeds, publish PWM values to ESCs
3. Create vehicle state node - track depth, orientation, velocity

This is very subject to change as we start making more progress

### Additional resources
**Link to ROS 2 documentation**

https://docs.ros.org/en/jazzy/index.html

**Link to ROS 2 Jazzy Installation Guide**

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

**Link to Northwestern's Master of Science in Robotics (MSR) website**

https://nu-msr.github.io

### Notes
Currently using ROS2 Jazzy, but might have to roll back to ROS2 Humble if needed

