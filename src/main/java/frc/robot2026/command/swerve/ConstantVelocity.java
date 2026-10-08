package frc.robot2026.command.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.swerve.DriveTrainInterface;

public class ConstantVelocity extends Command {
    final DriveTrainInterface drivetrain;
    final SwerveDriveKinematics kinematics;
    final SwerveModulePosition[] meas_pos; 

    final int Move_Delay = 10;  //frames to wait for wheels to align with heading

    final ChassisSpeeds park = new ChassisSpeeds(0.0, 0.0, 0.0);
    ChassisSpeeds moving;
    final Timer timer;
    final Rotation2d heading;

    SwerveModuleState[] out_states;
    double time;
    double velocity;
    int delay_count;


 public ConstantVelocity(double velocity, Rotation2d heading, double time) {
        drivetrain = RobotContainer.getSubsystem("drivetrain");        
        kinematics = drivetrain.getKinematics();
        meas_pos = drivetrain.getSwerveModulePositions();
              
        timer = new Timer();
        this.time = time;
        this.velocity = velocity;
        this.heading = heading;
    
        addRequirements(drivetrain);
        if (velocity*time > 2.0) {
            System.out.println("WARNING: openloop move over 2m are you sure? Ensure the robot has room.");
        }
    }

    public ConstantVelocity(double velocity, double time) {
        this(velocity, Rotation2d.kZero, time);
    }

@Override
    public void initialize() {
        delay_count = 0;        
        var x_vel = velocity * heading.getCos();
        var y_vel = velocity * heading.getSin();
        moving = new ChassisSpeeds(x_vel, y_vel, 0.0);
        // no motion to start, but move angles, reset distance meters for measuring at end
        out_states = kinematics.toSwerveModuleStates(park);
        for (int i = 0; i < out_states.length; i++) 
            out_states[i].angle = heading;   // set wheel rotation
            
        timer.stop();
    }

    @Override
    public void execute() {
        // wait Move_Delay frames to allow wheels to align, then go
        if (delay_count >= Move_Delay) {
            out_states = kinematics.toSwerveModuleStates(moving);
            timer.restart();
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
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return delay_count >= Move_Delay && timer.hasElapsed(time);
    }

}
