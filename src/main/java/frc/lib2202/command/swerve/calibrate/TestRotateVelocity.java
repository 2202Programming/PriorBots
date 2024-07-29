package frc.lib2202.command.swerve.calibrate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;

public class TestRotateVelocity extends Command {

    final SwerveDrivetrain drivetrain;
    final SwerveDriveKinematics kinematics;
    final SwerveModulePosition[] meas_pos;
    final double[] initial_positions;
    final Timer timer;

    SwerveModuleState[] out_states;
    double time;
    double omega; // [deg/s]

    ChassisSpeeds moving;
    Rotation2d heading0;

    public TestRotateVelocity(double omega, double time) {
        drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
        kinematics = drivetrain.getKinematics();
        meas_pos = drivetrain.getSwerveModulePositions();
        initial_positions = new double[meas_pos.length];
        timer = new Timer();
        addRequirements(drivetrain);
        heading0 = new Rotation2d(0);
        this.omega = omega / 57.3; // [rad/s]
        this.time = time;
    }

    @Override
    public void initialize() {
        drivetrain.setPose(new Pose2d(0.0, 0.0, heading0));
        var x_vel = 0.0;
        var y_vel = 0.0;
        moving = new ChassisSpeeds(x_vel, y_vel, omega);

        for (int i = 0; i < meas_pos.length; i++) {
            initial_positions[i] = meas_pos[i].distanceMeters;
        }
        timer.reset();
        out_states = kinematics.toSwerveModuleStates(moving);
    }

    @Override
    public void execute() {
        drivetrain.drive(out_states);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        timer.stop();
        report_results();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }

    public void report_results() {
        double expected_distance = omega * timer.get();
        System.out.println("ConstantVelocityTest ran for " + timer.get() + " seconds.");
        System.out.println("Expected distance: " + expected_distance + "[m]");
        for (int i = 0; i < initial_positions.length; i++) {
            double moved = meas_pos[i].distanceMeters - initial_positions[i];
            System.out.println("\tmodule[" + i + "] moved=" + moved + "[m] " +
                    "err= " + (moved - expected_distance) + "[m]");
        }
    }
}
