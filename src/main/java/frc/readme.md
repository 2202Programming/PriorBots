# PriorBots
This repo contains code that is updated each year with new vendor and support libraries 
to keep all robots running on the most recent code.

## Adding a Bot
Robots should be placed under 'robot<year>' folder. That code should only reference 
that its own path or any common library, like `lib2202` or `PathPlanner`.

Main.java is then modified to include and instantiate the RobotSpec_<year> object.

`new RobotSpec_<spec name or year>();` <br>
See **Main.java**

## Setting `serialnum` for debug
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
$env:serialnum ='031b7511'    #swerveBot aka Tim
$env:serialnum ='032D2062'    #compbot 2024
$env:serialnum ='03238151'    #Chadbot
$env:serialnum ='032381BF'    #alphabot 2024
$env:serialnum ='03415A8E'    #comp bot 2025
```
Pick one of these before debugging in simulation mode.
<br>
You can verify the env setting with:    
    `Get-ChildItem Env:serialnum`

# Helpful GIT commands
Git can be tricky. Basic operations can be done with <b>gitkraken</b> or a <b>VSCODE</b> plugin, but sometimes 
using the command line is useful.<br/>
[Git Docs](https://git-scm.com/docs/git)

## Status
```
git status 
git submodule status --recursive
````

## Code Updates
```
git switch <branchName>
git branch  <new branchName> 
git pull 
git pull  --rebase         #rebase your changes to keep linear commits
git commit -m'meaningful commit comment'
git push
```

To see what repos your connected to.
```
git remote -v
```

## GIT Submodules
Submodules need to be updated with their own commands. Add the ```--recursive``` when you want to
make sure all submodules are update,
```
git submodule update --init --recursive    # pull for the submodules
git submodule init  --recursive
git submodule status --recursive
git submodule update --recursive --remote
```

## Modify your GIT env 
These commands only need to be run once on your working git clone. It modifies your local ```.git/config``` file.
```
git config submodule.recurse true  # always use --recurse-submodules on a 'git pull'
```

