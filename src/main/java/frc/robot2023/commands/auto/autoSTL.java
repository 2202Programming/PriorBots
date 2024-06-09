package frc.robot2023.commands.auto;

import java.util.List;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlanner;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.base.RobotContainerOrig;
import frc.lib2202.subsystem.Swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;
import frc.lib2202.subsystem.hid.SwitchboardController.SBButton;

public class autoSTL extends Command {
    // Subsystems
    SwerveDrivetrain sdt = RobotContainerOrig.RC().drivetrain;
    HID_Xbox_Subsystem dc = RobotContainerOrig.RC().dc;

    // Constraints
    double maxVel = 3.0; // for mid going over charge station (STL elims) prev 4.5
    double macAccel = 3.0; // for mid going over charge station (STL elims) pre 4.0

    // Path info
    String pathName = "stl";
    SequentialCommandGroup cmd;
    List<PathPlannerTrajectory> pathGroup;

    public autoSTL() {
        addRequirements(sdt, dc);
    }

    @Override
    public void initialize() {
        // get location
        if (dc.readSideboard(SBButton.Sw11)) pathName += "Edge";
        else if (dc.readSideboard(SBButton.Sw12)) pathName += "Mid";
        else if (dc.readSideboard(SBButton.Sw13)) pathName += "Far";
        else pathName += "Mid";   //added for sim testing where no sideboard exists, npe otherwise.

        // if it's hail mary
        if (dc.readSideboard(SBButton.Sw15)) pathName += "HailMary";

        // if it's no balance
        if (!dc.readSideboard(SBButton.Sw21)) pathName += "NoBalance";

        // load the path group
        pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(maxVel, macAccel));

        // STL elims we're not that fancy
        // if (pathName.contains("Mid")) {
        //     if (dc.readSideboard(SBButton.Sw22)) {
        //         pathGroup.remove(1);
        //         pathGroup.remove(1); // assume index 1 is right pickup
        //     }
        //     else {
        //         pathGroup.remove(2); // assume index 2 is left pickup
        //         pathGroup.remove(2);
        //     }
        // }

        cmd = new SequentialCommandGroup(RobotContainerOrig.RC().autoBuilder.fullAuto(pathGroup));
        
        cmd.schedule();
    }

    @Override
    public void execute() {
        // do nothing
    }

    @Override
    public void end(boolean interrupted) {
        // do nothing
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
