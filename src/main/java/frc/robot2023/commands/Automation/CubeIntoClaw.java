package frc.robot2023.commands.Automation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.base.RobotContainerOrig;
import frc.robot2023.commands.Arm.ArmLockForDrivingFS;
import frc.robot2023.commands.Arm.SafeMoveToCubePickup;
import frc.robot2023.commands.EndEffector.InWheelsWithGate;
import frc.robot2023.subsystems.Claw_Substyem;

public class CubeIntoClaw extends SequentialCommandGroup {
    Claw_Substyem claw = RobotContainerOrig.RC().claw;

    public CubeIntoClaw() {
        addCommands(
            new SafeMoveToCubePickup(),
            new InWheelsWithGate(),
            new ArmLockForDrivingFS()
        );
        
    }
}
