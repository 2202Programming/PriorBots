package frc.robot2023.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.base.RobotContainerOrig;
import frc.robot2023.subsystems.Claw_Substyem;

public class TrackThenMove extends SequentialCommandGroup {
    // SSs
    Claw_Substyem claw = RobotContainerOrig.RC().claw;

    public TrackThenMove(CollectivePositions armPos, double wait) {
        addCommands(
                new InstantCommand(() -> {
                    claw.setTransitionClawTrackMode();
                }),
                new WaitCommand(wait),
                new MoveCollectiveArm(armPos));
    }
}
