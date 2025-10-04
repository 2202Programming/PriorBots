package frc.robot2025.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.Robot;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.OdometryInterface;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;
import frc.robot2025.commands.AlianceAwareSetPose;
import frc.robot2025.commands.DriveToPickupTag;
import frc.robot2025.commands.DriveToReefTag;
import frc.robot2025.commands.DropSequenceBaseCommands.ReleaseCoral;
import frc.robot2025.commands.DropSequenceBaseCommands.setElevatorSetpoint;
import frc.robot2025.commands.DropSequenceBaseCommands.setWristPos;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;
import frc.robot2025.subsystems.WristFLA;

/** Add your docs here. */
public class DeliveryCmdFactory {

    final OdometryInterface vpe;  //vision pose estimator
    final Elevator_Subsystem elevator; 
    final DriveTrainInterface sdt;

    // for coral eject 
    final int releaseCount = 25;

    //Factory is initialized by getting correct subsystems
    public DeliveryCmdFactory(String vpeName) {
        vpe = RobotContainer.getSubsystemOrNull(vpeName);
        elevator = RobotContainer.getSubsystemOrNull(Elevator_Subsystem.class);
        sdt = RobotContainer.getSubsystem("drivetrain");   
    }

    public Command DeliverReefStart(String cmdName,
            Pose2d startPose, int reefPosition, String reefSide,  String pickupSide,
            Levels eleLevel, String levelTrimName, double wristPos, 
            Command part2) {
        
        SequentialCommandGroup cmd = new SequentialCommandGroup();
        cmd.setName(cmdName);
        @SuppressWarnings("unchecked")
        AlianceAwareSetPose initPose = new AlianceAwareSetPose(startPose, vpe::setPose);
        DriveToReefTag toReef = new DriveToReefTag(reefSide, reefPosition);
        // might not need to pickup, example position 2
        Command toPickup = (pickupSide.startsWith("n")) ?
                new PrintCommand("no pickup") :
                new DriveToPickupTag(pickupSide);
        var eleCmd = ElevatorDelivery(eleLevel, levelTrimName, wristPos, releaseCount);
        if (eleLevel == Levels.LFour) {
            toReef.withDistanceScheduleCmd(eleCmd, 0.8);  //.8m worked 
            cmd.addCommands(initPose, toReef, toPickup);
        }
        else {
            // not doing l4, run eleCmd after we get there
            cmd.addCommands(initPose, toReef, eleCmd, toPickup);
        }        
        if (part2 != null) cmd.addCommands(part2);
        cmd.addRequirements(sdt);
        return cmd;
    }

    // deliver but don't set initial positons
    public Command DeliverReefFromPickup(String cmdName, double wait,
            int reefPosition, String reefSide, String pickupSide,
            Levels eleLevel, String levelTrimName, double wristPos) {
        
        SequentialCommandGroup cmd = new SequentialCommandGroup();
        cmd.setName(cmdName);
        var pickup = new WaitCommand(wait);
        DriveToReefTag toReef = new DriveToReefTag(reefSide, reefPosition);
        // might not need to pickup, example position 2
        Command toPickup = (pickupSide.startsWith("n")) ?
                new PrintCommand("no pickup") :
                new DriveToPickupTag(pickupSide);
        var eleCmd = ElevatorDelivery(eleLevel, levelTrimName, wristPos, releaseCount);
        if (eleLevel == Levels.LFour) {
            toReef.withDistanceScheduleCmd(eleCmd, 0.8);  //.8m worked 
            cmd.addCommands(pickup, toReef, toPickup);
        }
        else {
            // not doing l4, run eleCmd after we get there
            cmd.addCommands(pickup, toReef, eleCmd, toPickup);
        }           
        cmd.addRequirements(sdt);
        return cmd;
    }


    public Command ElevatorDelivery(Levels eleLevel, String levelTrimName, double wristPos, double releaseCount ) {
        if (elevator == null) return new PrintCommand("No elevator found.");
        Command cmd;
        if (eleLevel == Levels.LFour) {
        // we have an elevator make a real command
           cmd = new SequentialCommandGroup (
            //move elevator
            new ParallelCommandGroup(
                new setElevatorSetpoint(eleLevel, levelTrimName).withTimeout(2.0),
                new setWristPos(1.5, "L4")),
            new setWristPos(wristPos, levelTrimName),
            //eject coral
            new ReleaseCoral(),
            new setWristPos(1.5, "L4"),
            // return to pickup
            new ParallelCommandGroup(
                new setWristPos(WristFLA.PICKUP_POSITION, "pickup"),
                new setElevatorSetpoint(Levels.PickUp, "pickup")));
        }else {
            // L2, L3 
            cmd = new SequentialCommandGroup (
            new ParallelCommandGroup(
                new setElevatorSetpoint(eleLevel, levelTrimName),
                new setWristPos(WristFLA.MID_POSITION, levelTrimName)).withTimeout(2.0),
            new ReleaseCoral(),
            new ParallelCommandGroup(
                new setWristPos(WristFLA.PICKUP_POSITION, "pickup"),
                new setElevatorSetpoint(Levels.PickUp, "pickup")));

        }
        var cmd2 = new PrintCommand("doing elevator delivery:" + eleLevel.toString());
        if (Robot.isSimulation()) return cmd2;
        return cmd;
    }

}
