package frc.robot2023.commands.Automation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.base.RobotContainerOrig;
import frc.robot2023.commands.Arm.ArmLockForDrivingFS;
import frc.robot2023.commands.Arm.CollectivePositions;
import frc.robot2023.commands.Arm.MoveCollectiveArm;
import frc.robot2023.commands.EndEffector.CloseClawWithGate;
import frc.robot2023.commands.EndEffector.MoveWrist;
import frc.robot2023.subsystems.Claw_Substyem;
import frc.robot2023.subsystems.Claw_Substyem.ClawTrackMode;

public class AutoUprightConePickup extends SequentialCommandGroup {
    final Claw_Substyem claw = RobotContainerOrig.RC().claw;

    public AutoUprightConePickup() {
        addCommands(
            new InstantCommand(() -> {
                claw.open();
                // this below is veerrryyy dangerous without knowing the setpoint, assuming it's travel mode
                claw.setTrackElbowMode(ClawTrackMode.free);
            }),
            new MoveCollectiveArm(CollectivePositions.uprightConePickup),
            new MoveWrist(CollectivePositions.uprightConePickup.pos_info.wristPos),
            new CloseClawWithGate(),
            new WaitCommand(0.25),
            new MoveCollectiveArm(CollectivePositions.uprightConeTravelHalfway),
            new ArmLockForDrivingFS()
        );
    }
    
}
