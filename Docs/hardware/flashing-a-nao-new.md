# Flashing and Configuring a Nao V6

## Installing the Nao OS

We are currently using the `nao-2.8.5.11_ROBOCUP_ONLY_with_root.opn` operating system.

***This is a custom Nao OS for use with RoboCup ONLY. It is NOT PUBLICLY available. You may only obtain this file by being in the league***


To flash a Nao V6, you need to use the following scripts in the `Install` folder of the repo:

1. `createRootImage` eg. `sudo ./createRootImage <file location of nao-2.8.5.11_ROBOCUP_ONLY_with_root.opn>`
2. run from root `./Make/Common/createHomeArchive` once this is done move the created file to Install folder `home.tar.gz`
3. run `sudo ./Install/createOPN` This will produce a file called `image.opn`
4. `flash` this requires 'image.opn' and USB location, not partition eg. `sudo ./Install/flash image.opn /dev/sdb`

### How to determine flash block device (usb)

1. use command lsblk
2. check the location of the USB
3. use the command 'umount' followed by USB directory eg. '/dev/sdb'
4. now the flash command should work.

### First Time Flashing

The first time you flash a NAO you will need to first flash the official `nao-2.8.5.11_ROBOCUP_ONLY_with_root.opn`. This will make sure the firmware version for the motors are all set correctly. If you don't do this step you may encounter the following errors:

```bash
localhost hal[2256]: [E] 1707292123.729957 2269 DeviceMotorBoard::handleMotorBoardConfig: Device motorboard named LeftShoulderBoard has config version 11. Only 10 is managed.

localhost lola[2446]: [W] 1707292129.519688 2450 LoLA.behaviors.DiagnosisLegacy: Board LeftShoulderBoard, 196-->INTERNAL_ERROR-->NOT_INTIALIZED
```

You can use the manufacturers tool to setup a usb with the default root image: http://doc.aldebaran.com/2-1/software/naoflasher/naoflasher.html

### Flashing Process

Once you have the USB stick ready to go:

1. Insert the USB stick into the back of the robots head while the robot is off.
2. Press and hold down the button on the chest of the robot until it lights up blue.
3. Leave until it says `Ognak gnuk` or turns off(~18 minutes for a factory reset).

This is just a guideline - if this exact behaviour doesn't happen, there may not be anything wrong.

## Naming the Nao

The name of the robot is automatically determined using the configs outlined in the repo. The `/etc/hostname` and other fields specific to the robot will automatically be set on flash if configured correctly on the rego before the flash device was made.

You can edit/add robots by editing files in the folder TODO and reflashing a USB.

## Initial Code Setup

Our codebase auto-configures the rest of the robot software.
Run the following commands:

1. `nao_sync -r <robot>`. Enter the password for the robot when asked to, and enter the password (`nao`) when asked to. If "Host key verification failed" appears in your console, type 
`ssh-keygen -f "/home/swift/.ssh/known_hosts" -R <robot>.local` 

Reboot, or restart the Softbank `naoqi` program:

```bash
ssh nao@<robot.local>
# Optional - Try 4 chest button presses if WiFi is not working
sudo reboot
```

After reboot:

```bash
ssh nao@<robot.local>
nao restart (not always required)
./redbackbots
```

You can now run the RedBackBots code via chest button presses or by ssh-ing in as above.
Verify that that `redbackBots` works - your're done!