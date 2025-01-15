# PriorBots
This repo contains code that is updated each year with new vendor and support libraries 
to keep all robots running on the most recent code.

# Adding a Bot
Robots should be placed under Robot<year> folder. That code should only reference 
that its own path or any common library, like `lib2202` or `PathPlanner`.

Main.java is then modified to include and instantiate the RobotSpec_<year> object.

`new RobotSpec_<spec name or year>();` <br>
See **Main.java**

# Setting `serialnum` for debug
Serial numbers are read from an OS environment variable called `serialnum`.  It is set on
the roboRIO hardware for all robots. This number is used to select which RobotSpec_<>
is constructed.
<br>
When running in simulation for debugging, you must set the serialnum on your PC.
In the VSCode's Debug:Main power shell, used for starting the simulation, you must 
have serialnum set to select desired robot spec to use in debug. Use the $env: command 
to set.
<br>
In the Powershell:
```
$env:serialnum ='031b7511'          #swerveBot aka Tim
$env:serialnum ='032D2062'          #compbot 2024
$env:serialnum ='03238151'          #Chadbot
$env:serialnum ='032381BF'          #alphabot 2024
```
Pick one of these before debugging in simulation mode.
<br>
You can verify the env setting with:    
    `Get-ChildItem Env:serialnum`