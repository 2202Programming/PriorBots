/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot2019.commands.arm;

import static frc.lib2202.Constants.DT;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.commands.util.MathUtil;
import frc.robot2019.commands.util.RateLimiter;
import frc.robot2019.commands.util.RateLimiter.InputModel;
import frc.robot2019.subsystems.ArmSubsystem;
import frc.robot2019.subsystems.ArmSubsystem.Position;

public class MoveArmToPosition extends WaitCommand {
    double timeout;
    double height;
    double projection;
    double error;
    final private ArmSubsystem arm;

    // Phyical values from sub-systems as needed
    private Position armPosition;
    private RateLimiter heightLimiter;
    private RateLimiter projectionLimiter;

    public MoveArmToPosition(double height, double projection, double error, double timeout) {
        super(timeout);
        arm = RobotContainer.getSubsystem(ArmSubsystem.class);
        addRequirements(arm);
        this.height = height;
        this.projection = projection;
        this.timeout = timeout;
        this.error = Math.abs(error);

        projectionLimiter = new RateLimiter(DT, () -> this.projection, // inputFunc gripperX_cmd
                arm::getProjection, // phy position func
                arm.MIN_PROJECTION, // output min
                arm.MAX_PROJECTION, // output max
                -50.0, // inches/sec // falling rate limit
                50.0, // inches/sec //raising rate limit
                InputModel.Position);

        heightLimiter = new RateLimiter(DT, () -> this.height, // gripperH_cmd var as set by this module
                arm::getHeight, // phy position func
                ArmStatePositioner.kHeightMin, // output min
                ArmStatePositioner.kHeightMax, // output max
                -80.0, // inches/sec // falling rate limit
                80.0, // inches/sec //raising rate limit
                InputModel.Position);
    }

    @Override
    public void initialize() {
        super.initialize(); //setTimeout(timeout);
        heightLimiter.initialize();
        projectionLimiter.initialize();
    }

    @Override
    public void execute() {
        heightLimiter.execute();
        projectionLimiter.execute();
        double h_cmd = heightLimiter.get();
        double x_cmd = projectionLimiter.get();
        double heightAbovePivot = h_cmd - arm.ARM_PIVOT_HEIGHT;

        // Adjusts x_cmd so h_cmd is always reached
        if (heightAbovePivot * heightAbovePivot + x_cmd * x_cmd >= arm.MAX_ARM_LENGTH * arm.MAX_ARM_LENGTH) {
            // Assumes h_cmd doesn't go above the max attainable height
            x_cmd = Math.sqrt(arm.MAX_ARM_LENGTH * arm.MAX_ARM_LENGTH - heightAbovePivot * heightAbovePivot);
        }

        // Calculate the angle
        double calculatedAngle = 0.0;
        if (Math.abs(heightAbovePivot) >= 1e-6) {
            calculatedAngle = (90 - Math.toDegrees(Math.atan(heightAbovePivot / Math.abs(x_cmd)))) * Math.signum(x_cmd);
        }
        double curAngle = MathUtil.limit(calculatedAngle, arm.PHI_MIN, arm.PHI_MAX);

        // Calculate extension based on current angle
        double calculatedExtension = arm.STARTING_EXTENSION;
        if (Math.abs(curAngle) >= 1e-6) {
            calculatedExtension = (x_cmd / Math.sin(Math.toRadians(curAngle))) - arm.ARM_BASE_LENGTH - arm.WRIST_LENGTH;
        }
        // Limiting here is technically unnecessary because limiting is also done in
        // setExtension
        double extensionLength = MathUtil.limit(calculatedExtension, arm.EXTEND_MIN, arm.EXTEND_MAX);

        arm.setAngle(curAngle);
        arm.setExtension(extensionLength);
    }

    @Override
    public boolean isFinished() {
        armPosition = arm.getArmPosition();
        double h_err = Math.abs(armPosition.height - height);
        double x_err = Math.abs(armPosition.projection - projection);
        boolean posGood = (h_err < error) && (x_err < error);
        return posGood || super.isFinished();
    }

    public void setHeightLimiter(double minHeight, double maxHeight, double fallSpeed, double raiseSpeed) {
        heightLimiter.setConstraints(minHeight, maxHeight, fallSpeed, raiseSpeed);
    }

    public void setProjectionLimiter(double minProjection, double maxProjection, double retractSpeed,
            double extendSpeed) {
        projectionLimiter.setConstraints(minProjection, maxProjection, retractSpeed, extendSpeed);
    }

    public RateLimiter getHeightLimiter() {
        return heightLimiter;
    }

    public RateLimiter getProjectionLimiter() {
        return projectionLimiter;
    }
}