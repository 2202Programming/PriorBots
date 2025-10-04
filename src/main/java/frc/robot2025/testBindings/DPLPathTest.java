package frc.robot2025.testBindings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.pathing.MoveToPose;
import frc.lib2202.command.swerve.AllianceAwareGyroReset;
import frc.lib2202.command.swerve.RobotCentricDrive;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.hid.HID_Subsystem;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2025.commands.DriveToPickupTag;
import frc.robot2025.commands.DriveToReefTag;
import frc.robot2025.commands.ScaleDriver;
import frc.robot2025.subsystems.SignalLight;

public final class DPLPathTest {

    static OdometryInterface odo;
    static String OdometryName = "vision_odo"; //VisionPoseEstimator.class.getSimpleName();
    static DriveTrainInterface sdt;
    static SignalLight signal;
    static HID_Subsystem dc;

    public static void myBindings(HID_Subsystem _dc) {
        dc = _dc;
        odo = RobotContainer.getObjectOrNull(OdometryName);  // or "odometry"
        sdt = RobotContainer.getObjectOrNull("drivetrain");
        signal = RobotContainer.getObjectOrNull("signal");
        
        // get an xbox controller for the operator, or null
        CommandXboxController opr = (dc.Operator() instanceof CommandXboxController)
                ? (CommandXboxController) dc.Operator()
                : null;

        CommandXboxController driver = (dc.Driver() instanceof CommandXboxController)
                ? (CommandXboxController) dc.Driver()
                : null;

        // if we have what we need, create our commands
        // signal light object is protected in the bound command       
        if (odo != null && sdt != null && driver != null) {
            xboxDriver(driver);
        }
        if (odo != null && sdt != null && opr != null) {
            xboxOperator(opr);        
        }
    }

    static void xboxDriver(CommandXboxController driver) { 
        // minimal standard driver controls - gyro reset and robot centric
        driver.y().onTrue(new AllianceAwareGyroReset(true));
        driver.rightBumper().whileTrue(new RobotCentricDrive(sdt, dc));

        //other binding used by driver from comp bindings
        // Driver will wants precision robot-centric throttle drive on left trigger
        driver.leftBumper().whileTrue(new ParallelCommandGroup(
                    new ScaleDriver(0.3),
                    new RobotCentricDrive(sdt, dc)));
        //drive to reef
        driver.leftTrigger().whileTrue(new DriveToReefTag("left"));
        driver.rightTrigger().whileTrue(new DriveToReefTag("right"));

        //drive to pickup
        driver.povLeft().whileTrue(new DriveToPickupTag("left"));
        driver.povRight().whileTrue(new DriveToPickupTag("right"));
    }

    static void xboxOperator(CommandXboxController opr) {
        // my testing commands

        opr.a().onTrue(new InstantCommand( () ->{
            ///WIP - clear odo history and sdt - keep last estimate as set point
            //use when robot not moving, clearing odo,sdt history
            Pose2d vision_current = odo.getPose();
            //var vel = sdt.getChassisSpeeds();
            odo.printPose();            
            sdt.setPositions(0); //clear wheel positions,
            ///Pose2d newPose = new Pose2d(0.45, 1.70, odo.getPose().getRotation());
            odo.setPose(vision_current);
        }));


        // test moveToPose - 1m forward in field coords
        opr.povUp().onTrue(new InstantCommand(() -> {
            Pose2d currentPose = odo.getPose(); // field coords
            // add 1m forward, field not robot.
            Pose2d target = new Pose2d(currentPose.getX() + 1.0,
                    currentPose.getY(), currentPose.getRotation());
            // force a color
            if (signal != null)
                signal.setLight(SignalLight.Color.BLUE);
            // calc path
            Command cmd = new MoveToPose(OdometryName, target);
            // turn signal off after our move, if we have a signal object
            if (signal != null)
                cmd = cmd.andThen(signal.getColorCommand(SignalLight.Color.OFF));
            cmd.setName("moveto-fwd/w signal"); 
            cmd.schedule();
        }));

        // calc and execute a path - 1m forward in X field coords
        opr.povDown().onTrue(new InstantCommand(() -> {
            Pose2d currentPose = odo.getPose(); // field coords
            // add 1m forward, field not robot.
            Pose2d target = new Pose2d(currentPose.getX() - 1.0,
                    currentPose.getY(), currentPose.getRotation());
            // calc path
            Command cmd = new MoveToPose(OdometryName, target);      
            cmd.setName("moveto-backup");     
            cmd.schedule();
        }));
        
    }
   
}