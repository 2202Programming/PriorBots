package frc.robot2019.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.commands.CommandManager.Modes;
import frc.robot2019.commands.arm.MoveArmToRawPosition;
import frc.robot2019.commands.drive.DriveByPowerAndJoystickCommand;
import frc.robot2019.commands.drive.HABDriveByPowerAndJoystickCommand;
import frc.robot2019.commands.intake.WristSetAngleCommand;
import frc.robot2019.commands.intake.WristTrackAngle;
import frc.robot2019.commands.util.Angle;
import frc.robot2019.subsystems.ClimberSubsystem;

public class Level2ClimbGroup extends SequentialCommandGroup {
    public Level2ClimbGroup(double climbHeight, double retractHeight) {
        double timeToDriveForward = 30.0;
        double rollPower = 0.6;
        double drivePower = 0.4; // Positive power goes to negative direction

        ClimberSubsystem climber = RobotContainer.getSubsystem(ClimberSubsystem.class);

        //if separate command to bring up robot change to parallel
        addCommands(climber.zeroSubsystem(),   //hack to zero counters
            new WristSetAngleCommand(0),
            new MoveArmToRawPosition(90, 12, 0.5, 180),
            new PawlSureFire(Robot.climber.Extend, 4),
            new DeployClimbFoot(1.0, climbHeight),    // 20.5 uses limit switch
            new WaitCommand(0.5));

        ParallelCommandGroup forwardCmds = new ParallelCommandGroup();
        forwardCmds.setName("going forward");
        forwardCmds.addCommands(
            new ClimbRollForward(0.2, rollPower, 0.3),   // power, timeout
            new HABDriveByPowerAndJoystickCommand(drivePower, 0.25, 0.6)); // power, timeout
        
        addCommands(forwardCmds);
        SequentialCommandGroup forwardCmds3 = new SequentialCommandGroup(); 
        forwardCmds3.setName("Going forward 2");
        forwardCmds3.addCommands(
            new PawlSureFire(Robot.climber.Retract,  6),
            new ParallelCommandGroup(
                new DeployClimbFoot(-1.0, retractHeight),    // neg power retract / limit sw
                new DriveByPowerAndJoystickCommand(drivePower, 0.25, 0.6, timeToDriveForward))); // neg power drive reverse
        
        addCommands(
            new MoveArmToRawPosition(-90, 12, 0.6, 180),
            forwardCmds3);

        addSequential(new MoveArmToRawPosition(-90, 6, 0.6, 180));
        addParallel(new WristTrackAngle(Angle.Back_Perpendicular_Down.getAngle()));
        addSequential(new DriveByPowerAndJoystickCommand(drivePower, 0.25, 0.6, 200.0));
    }

    /*
    steps for climb: * - means I attemped but probably needs more
    driver drives back into hab
    engage pawl piston - keep engaged *
    run m23 until we reach 19 inches - there is an encoder *
    stop m23 *
    engage drawer slide piston *
    run drive wheels - crawling backwards
    run m22 - open loop until drawer slides in *
    disengage drawer slide piston *
    stop drive wheels
    stop m22 *
    disengage pawl piston *
    run m23 backwards until climber is mostly up *
    end

    pawl piston is foot, m23 is foot extension motor, m22 is motor to move foot
    */

    @Override
    public void interrupted() {
        Robot.driveTrain.stop();
        Robot.climber.setRollerSpeed(0.0);
        Robot.climber.setExtenderSpeed(0.0);
        Robot.climber.setPawl(Robot.climber.Retract);
        Robot.climber.setDrawerSlide(Robot.climber.HoldSlide);
        Robot.m_cmdMgr.setMode(Modes.Drive);
    }

    public void end(boolean interrupted) {

    }
}