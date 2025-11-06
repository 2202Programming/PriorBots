In PriorBots, robots have their deploy folders designated by year or robot name.  Under that folder
is the original deploy directory.  This not only keeps the data year to year, but the paths or 
deploy details may be still used.

The PriorBot image supports multiple robots, so you don't know which code or deploy folder is 
At runtime, the robot's correct deployneeded at runtime. Because of this, everything in the deploy subtree is copied to the Rio. 
 folder is copied down to correct directory when the
robot is constructed.

Dir Structure
==============
deploy/
	2025/deploy/
		pathplanner/...
	2024/deploy/
		pathplanner/...

	ChadBoot/deploy/...

At runtime, specifically during robot init, the instantiated robot supplies the folder name and then 
the init code will copy the files from robot specific deploy folder to the root deploy folder. This
keeps the deploy tree, and thus the paths, in the expected locations.

To edit the paths/autos in the priorbot repo, open pathplanner and select the year/robot folder you want to edit.
Pathplanner will find deploy/pathplanner/ and edits may be made.


For the copy to work, you must once telnet into the roborio and change the owner of /home/lvuser/deploy from admin
to lvuser.  Only needed once or when a roborio is reimaged because deploy folder is owned by admin.

Telnet/putty/mobaxterm into Roborio via USB  172.22.ll.2  on admin, pw is blank
cd /home/lvuser
chmod -R deploy lvuser

Once done, deploy copies will work with the added robot specific layer.
