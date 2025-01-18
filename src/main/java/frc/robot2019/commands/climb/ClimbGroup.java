package frc.robot2019.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot2019.Robot;
import frc.robot2019.commands.CommandManager.Modes;
import frc.robot2019.commands.arm.MoveArmToPosition;
import frc.robot2019.commands.arm.MoveArmToRawPosition;
import frc.robot2019.commands.drive.DriveByPowerAndJoystickCommand;
import frc.robot2019.commands.drive.HABDriveByPowerAndJoystickCommand;
import frc.robot2019.commands.intake.VacuumCommand;
import frc.robot2019.commands.intake.WristSetAngleCommand;
import frc.robot2019.commands.intake.WristTrackAngle;
import frc.robot2019.commands.util.Angle;

public class ClimbGroup extends SequentialCommandGroup {
    public ClimbGroup(double climbHeight, double retractHeight) {
        double timeToDriveForward = 30.0;
        double rollPower = 0.6;
        double drivePower = 0.4; // Positive power goes to negative direction

        // Climb Sequence
        addCommands(
            new ClimbZero(),  //Robot.climber.zeroSubsystem(),   //hack to zero counters
            new VacuumCommand(false, 0.04),
            new WristSetAngleCommand(-90),
            new MoveArmToRawPosition(-90, 12, 0.5, 180),
            new PawlSureFire(Robot.climber.Extend, 4),
            new DeployClimbFoot(1.0, climbHeight),   // 20.5 uses limit switch
            new WaitCommand(0.5));

        ParallelCommandGroup forwardCmds = new ParallelCommandGroup();
        forwardCmds.setName("Going forward");
        forwardCmds.addCommands(
            new ClimbRollForward(0.15, rollPower, 0.2),         // power, timeout
            new HABDriveByPowerAndJoystickCommand(drivePower, 0.25, 0.6)); // power, timeout
        

        // CommandGroup skidGroup = new CommandGroup("Going foward with skid");
        // skidGroup.addSequential(new WaitCommand(1.5));
        // skidGroup.addSequential(new MoveArmToRawPosition(-133, 19.0, 0.5, 180));
       
        
        // Drive into HAB Sequence
        addCommands(
            new MoveArmToRawPosition(-135, 20.0, 0.5, 180),        
            forwardCmds);

        SequentialCommandGroup forwardCmds3 = new SequentialCommandGroup();
        forwardCmds3.setName("Going forward 2");
        forwardCmds3.addCommands(new PawlSureFire(Robot.climber.Retract,  6),
            new ParallelCommandGroup(
                new DeployClimbFoot(-1.0, retractHeight),    // neg power retract / limit sw
                new DriveByPowerAndJoystickCommand(drivePower, 0.25, 0.6, timeToDriveForward))); // neg power drive reverse
        
        // Retract Sequence
        addCommands(
            new WristSetAngleCommand(0),
            new MoveArmToRawPosition(-90, 14, 0.6, 180),
            forwardCmds3);

        //Drive fully onto HAB sequence 
        addCommands(
            new MoveArmToRawPosition(-90, 6, 0.6, 180),
            new WristTrackAngle(Angle.Back_Perpendicular_Down.getAngle()));
            new DriveByPowerAndJoystickCommand(drivePower, 0.25, 0.6, 200.0);

        //TODO - test if this works, otherwise construct with static factory function and decorate.
        this.andThen(new InstantCommand( ()-> on_interrupted()));
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

    //TODO - can't override end() it is final on cmd groups, really each cmd should clean up
    void on_interrupted() {
        Robot.driveTrain.stop();
        Robot.climber.setRollerSpeed(0.0);
        Robot.climber.setExtenderSpeed(0.0);
        Robot.climber.setPawl(Robot.climber.Retract);
        Robot.climber.setDrawerSlide(Robot.climber.HoldSlide);
        Robot.m_cmdMgr.setMode(Modes.Drive);
    }

}