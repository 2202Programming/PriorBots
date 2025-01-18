package frc.robot2019.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.Robot;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.commands.CommandManager.Modes;
import frc.robot2019.commands.util.Angle;
import frc.robot2019.subsystems.ArmSubsystem;
import frc.robot2019.subsystems.IntakeSubsystem;

public class WristStatePositioner extends Command {
    private double curAngle;
    private Modes prevMode;
    private int prevIndex;

    // 3D tensor of angles: format is [state][inversionStatus][index]
    private double[][][] angles = { { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Construction
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Setting Zeros
            { { Angle.Starting_Hatch_Hunt.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // HuntGameStart
            { { Angle.Hatch_Pickup.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // HuntingHatch
            { { Angle.Perpendicular_Down.getAngle() }, { Angle.Back_Perpendicular_Down.getAngle() } }, // HuntingCargo
            { { Angle.Perpendicular_Down.getAngle() }, { Angle.Back_Perpendicular_Down.getAngle() } }, // HuntingFloor
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Capturing
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Recapturing
            { { Angle.Parallel.getAngle(), Angle.Parallel.getAngle(), Angle.Parallel.getAngle() },
                    { Angle.Back_Parallel.getAngle(), Angle.Back_Parallel.getAngle() } }, // Drive
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Defense
            { { Angle.Hatch_Delivery.getAngle(), Angle.Hatch_Delivery.getAngle(), Angle.Hatch_Delivery.getAngle() },
                    { Angle.Back_Hatch_Delivery.getAngle(), Angle.Back_Hatch_Delivery.getAngle(),
                            Angle.Back_Hatch_Delivery.getAngle() } }, // DeliverHatch
            { { Angle.Cargo_Delivery.getAngle(), Angle.Cargo_Delivery.getAngle(), Angle.Cargo_Delivery.getAngle() },
                    { Angle.Back_Cargo_Delivery.getAngle(), Angle.Back_Cargo_Delivery.getAngle(),
                            Angle.Back_Cargo_Delivery.getAngle() } }, // DeliverCargo
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Flipping
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, //
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, //
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, //
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, //
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, //
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, //
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, //
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Releasing
            { { Angle.Parallel.getAngle() }, { Angle.Back_Parallel.getAngle() } }, // Climbing
    };

    final IntakeSubsystem intake;
    final ArmSubsystem arm;

    public WristStatePositioner() {
        intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
        arm = RobotContainer.getSubsystem(ArmSubsystem.class);
        addRequirements(intake);
    }

    @Override
    public void execute() {
        // Update position based on current mode
        Modes curMode = Robot.m_cmdMgr.getCurMode();
        int invert = Robot.arm.isInverted() ? 1 : 0;
        int index = Robot.m_cmdMgr.getPositionIndex();
        if (curMode != prevMode || index != prevIndex) {
            // Update position only if state changes to allow something to override position
            // for that state
            curAngle = angles[curMode.get()][invert][index];
            prevMode = curMode;
            prevIndex = index;
        }

        // intake angle is relative to arm
        double offset = arm.getRealAngle() - curAngle;
        intake.setAngle(offset);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    //TODO - interrupted f() removed, test this new end()
    @Override
    public void end(boolean interrupted) {
        if (interrupted) interrupted();
    }

    //@Override  TODO test this idiom
    public void interrupted() {
        // Update position based on current mode
        Modes curMode = Robot.m_cmdMgr.getCurMode();
        int index = Robot.m_cmdMgr.getPositionIndex();
        if (curMode != prevMode || index != prevIndex) {
            // Update position only if state changes to allow something to override position
            // for that state
            prevMode = curMode;
            prevIndex = index;
        }
    }
}