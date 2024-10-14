# Configurations

In this page are notable configuration files that affect the robots on runtime.

## Game Config - redbackbots.cfg

**Location:** `Install/NaoHome/Config/redbackbots.cfg`

Contains the configuration for most in-game settings and contains the configurations for the following:

* **[stateestimation]** - Initial coordinates and referee settings

* **[behaviour]** - Skills used when starting up and positioning settings
  
* **[motion]** - Settings related to the getup skill

* **[kick]** - Variables for stability of kicks. *Note that this can be overwritten with personal robot settings; refer to next section.*

* **[vision]** - Variables for configuration of vision processing

* **[gamecontroller], [player]** - Base config for GameController and Player. *Refer to the next section instead.*

* **[camera]** - Controls camera settings such as exposure, color balance, and focus

**Notable:** The line `[behaviour] skill=Game` controls which behavior the robot displays when started with the chest taps. By default, it is Game, which is to play the game, but this can be changed as needed.

## Personalized Robot Configs

**Location:** `Config/Robots/Robotname`

Contains various personalized configuration files for each robot:

* **Body:** May contain personalized **[kick]**, which determines the lean offsets and default kicking foot

* **Head:** May contain personalized **[vision]** settings.

* **Network:** Contains robot's `name` along with `lan` and `wlan` IP.

* **Robot:** Contains personalized **[player]** settings, which has: `number` - The player number, `team` - The player's team number, `playerIP` - IP should be the same as in Network, `bodyName`, and `headName` - of which both should be the same as the name in Network.