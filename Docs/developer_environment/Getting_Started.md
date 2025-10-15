# Setup

## Virtual Machine Preparation

### Choose a VM provider

#### **Windows:**

* [VMWare][0]
* [VirtualBox][4]

If you have used WSL before, that is also an option, but it is not recommended if you're not comfortable with Linux. Extra steps for WSL are documented [here.](Working_with_WSL.md)

#### **MacOS:**

* [VMWare Fusion][0]
* [VirtualBox][4]
* [Parallels][3]
* [UTM][5]

[VirtualBox][4] is also required if you need to compile software in the Aldebaran-provided NaoVM.

## Virtual Machine Setup

### Setting up your OS

The current OS that works is [Ubuntu 22.04][14] (the long-term release one).
> Note: Apple silicon (eg M1) Macs require the ARM version of Ubuntu, which is only available from the [Daily Build images][15].

You can run the OS natively on its own partition, or in a VM.

It is recommended to have at least **30GB storage** partition and **3GB memory** otherwise the build script `Make/Common/buildSetup` will not work.

It is imperative to defer installing updates during and after the setup of the OS on your VM. This may break compatibility so please stay on v22.04.

You can find some prebuilt images on the redbackbots teams.

### Initial packages

We will need to install the following packages:

* git
* gitk
* vim
* vim-gtk3
* python3
* python3-pip
* python3-apt

And we'll need to remove the following unnecessary package:

* LibreOffice

You can do all this via the following commands:

```bash
sudo apt update
sudo apt install git gitk vim vim-gtk3 python3 python3-pip python3-apt
sudo apt purge libreoffice*
sudo apt autoremove
```


## Cloning the Repository

Use the following command to clone the code repository:

```bash
git clone --depth 1 --branch coderelease2025 https://github.com/rmit-computing-technologies/redbackbots-coderelease.git
```

## Run the Build Script

Now you are ready to run the build script. Use the following two commands to start this process:

```bash
cd ~/redbackbots
./Make/Common/buildSetup
```

It will ask you various questions after running the second command and best to answer yes (Y) to everything. This can take a while on the first go, around 30 minutes, but keep an eye on it as you'll need to respond to the prompts along the way.

This will:

* Set up your .bashrc with some robocup environment variables.
* Add your ssh key to the image on the robots.
* Setup /etc/hosts so that you can access robots via name when using ssh
* Generate a CMake project to build from
* Do an initial release build.

Once finished, close all the open terminals and you are now ready to run the robots.

Our public code release comes only with a default robot configuration. You may need to use the `CreateRobot` script to add a new robot.
```bash
./Make/Common/createRobot -r <ROBOT NAME> -i <LAST OCTET OF IP ADDRESS > -b <BODY SERIAL NUM> -s <HEAD SERIAL NUM>
```

You may also want to modify:
* The `"basePlayerIP"` value in `Config/defaults.cfg`
* placeholders values in `Install/Profiles/RBB`
* ssid & password details in `Install/Files/default_dhcp_example.yaml` & `Install/Files/default_static_example.yaml`

to match your network setup.

Checkout how to [flash a robot with our codebase](../hardware/flashing-a-nao-new.md) & [sync and run the code](../running_a_game/running-software.md)

[0]:     https://www.vmware.com/products/workstation-player.html
[3]:     https://www.parallels.com/au/
[4]:     https://www.virtualbox.org/wiki/Downloads
[5]:     https://docs.getutm.app/installation/macos/
[14]:    https://releases.ubuntu.com/22.04/
[15]:    https://cdimage.ubuntu.com/jammy/daily-live/current/
