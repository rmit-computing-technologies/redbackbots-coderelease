# Let's get started :zap:

## Syncing with the robot

Make sure you've run the build command at least once `./Make/Common/buildSetup`. Run the following command:

```bash
nao_sync -a -r ROBOTNAME
```

Note: *ROBOTNAME* should be in all lowercase for the command.

You will replace *ROBOTNAME* with the name found on the back of the robot's head. Once this has finished, you'll see `nao_sync done` in the terminal and the robot is ready to go.

From here you have two options to run the robot:

1. Run a simple command via the command line on the robot
2. Run the robot in Game mode and use GameController to let the robot(s) play a game by itself.

### 1. Run a simple command

To have the robot run a simple command, use the following commands to make this happen:

```bash
ssh nao@ROBOTNAME 
# replace ROBOTNAME with name of robot or the robots IP address
```

On the command line of the robot (you should see the terminal say 'nao@ROBOTNAME:~$'), run:

```bash
./redbackbots -s COMMAND
```

Here you will replace *COMMAND* with any of the behaviours or skills found in the following folders:

* Src/behaviours/body/skills
* Src/behaviours/body/roles
* Src/behaviours/body/test

For example:

```bash
./redbackbots -s Stand
# A simple skill where the robot simply stands up
# This will run the code in Src/behaviours/body/skills/Stand.py
```

After running the above command, and once you see the command line running, you will then press the chest button once, and the robot will stand up and start the given command.

**Note: As a recommendation, run `./redbackbots -s Demo` if this is your first time. The robot will simply find the ball and kick it until stopped.**

To safely stop the robot, place your hand on its head touching the 3 head buttons and the robot will sit down (*To stop the code from running, you can swipe the head buttons from front to back*). If running from the command line, you can also use Control + C to stop the code from running which again will make the robot stop and sit down.


## Further Reading

For more information on how to turn on / turn off Nao, please check out [Nao Button Interface](../hardware/Nao_Button_Interface.md)

