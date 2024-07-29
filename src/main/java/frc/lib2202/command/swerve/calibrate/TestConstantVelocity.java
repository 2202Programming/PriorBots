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

public class TestConstantVelocity extends Command {
    final SwerveDrivetrain drivetrain;
    final SwerveDriveKinematics kinematics;
    final SwerveModulePosition[] meas_pos; 
    final double[] initial_positions;

    final int Move_Delay = 25;
    final Timer timer;
    final Rotation2d heading0;

    SwerveModuleState[] out_states;
    double time;
    double velocity;
    int delay_count;

    final ChassisSpeeds park = new ChassisSpeeds(0.0, 0.0, 0.0);
    ChassisSpeeds moving;

    public TestConstantVelocity(double velocity, double time, double heading) {
        drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
        kinematics = drivetrain.getKinematics();
        meas_pos = drivetrain.getSwerveModulePositions();
        initial_positions = new double[meas_pos.length];
        addRequirements(drivetrain);
        timer = new Timer();
        this.time = time;
        this.velocity = velocity;
        heading0 = new Rotation2d(heading);

        if (velocity*time > 10.0) {
            System.out.println("WARNING: Test will move over 10m are you sure? Ensure the robot has room.");
        }
    }
    public TestConstantVelocity(double velocity, double time) {
        this(velocity, time, 0.0);
    }

    @Override
    public void initialize() {
        delay_count = 0;
        drivetrain.setPose(new Pose2d(0.0, 0.0, heading0));
        var x_vel = velocity * heading0.getCos();
        var y_vel = velocity * heading0.getSin();
        moving = new ChassisSpeeds(x_vel, y_vel, 0.0);
        // no motion to start, but move angles, reset distance meters for measuring at end
        out_states = kinematics.toSwerveModuleStates(park);
        for (int i = 0; i < out_states.length; i++) out_states[i].angle = heading0;
        for (int i = 0; i < meas_pos.length; i++) {
             initial_positions[i] = meas_pos[i].distanceMeters;
        }
        timer.reset();
    }

    @Override
    public void execute() {
        // wait Move_Delay frames to allow wheels to align, then go
        if (delay_count >= Move_Delay) {
            out_states = kinematics.toSwerveModuleStates(moving);
            timer.start();
        } else {
            delay_count++;
        }
        drivetrain.drive(out_states);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        out_states = kinematics.toSwerveModuleStates(park);
        drivetrain.drive(out_states);
        timer.stop(); 
        report_results();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }

    public void report_results(){
        double expected_distance = velocity * timer.get();
        System.out.println("ConstantVelocityTest ran for " + timer.get() + " seconds.");
        System.out.println("Expected distance: "+ expected_distance + "[m]");
        for(int i=0; i < initial_positions.length; i++) {
            double moved = meas_pos[i].distanceMeters - initial_positions[i];
            System.out.println("\tmodule["+ i +"] moved="+ moved +"[m] " +
            "err= " + (moved - expected_distance)+"[m]" );
        }
    }
}
