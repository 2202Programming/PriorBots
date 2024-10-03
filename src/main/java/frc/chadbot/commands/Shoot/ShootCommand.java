package frc.chadbot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.chadbot.RobotContainer;
import frc.chadbot.subsystems.Intake_Subsystem;
import frc.chadbot.subsystems.Magazine_Subsystem;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem;

public class ShootCommand extends Command{
    final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    final Shooter_Subsystem shooter;
    int count;
    int initialCount = 0;

    enum Stage{
        DoNothing,
        WaitingForSolution,
        BackingMagazine,
        WaitingForFlyWheel,
        Shooting,
    }Stage stage;
    public ShootCommand(){
        this.intake = RobotContainer.RC().intake;
        this.shooter = RobotContainer.RC().shooter;
        this.magazine = RobotContainer.RC().magazine;
    }

    public ShootCommand withInitialCargo(int count) {
        this.initialCount = count;
        return this;
    }

    @Override
    public void initialize(){
        stage = Stage.DoNothing;
        count = initialCount;
    }
    @Override
    public void execute(){
        switch(stage){
            case DoNothing:
                magazine.driveWheelOn(0.001);
                stage = Stage.BackingMagazine;
            break;
            case BackingMagazine:
                if(count >= 2){
                    magazine.driveWheelOff();
                    stage = Stage.WaitingForSolution;
                }
                count++;
            break;
            case WaitingForSolution:
                // TODO make sure shooterSettings get issued to shooter.
                stage = Stage.WaitingForFlyWheel;
            break;
            case WaitingForFlyWheel:
                if(shooter.isReadyToShoot()){
                    magazine.driveWheelOn(0.1);
                    stage = Stage.Shooting;
                }
            break;
            case Shooting:
                if(!shooter.isReadyToShoot()){
                    magazine.driveWheelOff();
                    stage = Stage.WaitingForFlyWheel;
                    count--;
                }
            break;
        }
        count++;
    }

    @Override
    public void end(boolean interrupted){
        stage = Stage.DoNothing;
        magazine.driveWheelOff();
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
