package frc.chadbot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.chadbot.subsystems.Intake_Subsystem;
import frc.chadbot.subsystems.Magazine_Subsystem;
import frc.chadbot.subsystems.shooter.FlyWheelRPM;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RPMShootCommand extends Command{ 
    public static final double USE_CURRENT_ANGLE = 0.0;

    final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    final Shooter_Subsystem shooter;
    final double TESTANGLE = 0.0;
    final double TESTTOL = 0.02;
    int ballCount = 999;
    int backupCounter = 0;

    NetworkTable table;
    NetworkTableEntry shooterState;

    FlyWheelRPM cmdRPM;
    FlyWheelRPM prevRPM;

    private boolean finished = false;

    final static FlyWheelRPM defaultShooterRPMs = new FlyWheelRPM(1000,1000);

    public enum Stage{
        DoNothing("Do Nothing"),
        WaitingForFlyWheel("Waiting for flywheel"),
        PreparingToShoot("Preparing to Shoot"),
        Shooting("Shooting");

        String name;

        private Stage(String name){
            this.name = name;
        }

        public String toString(){
            return name;
        }
    }
    
    Stage stage;

    public RPMShootCommand(FlyWheelRPM target){
        this.intake = RobotContainer.getSubsystem(Intake_Subsystem.class);
        this.shooter = RobotContainer.getSubsystem(Shooter_Subsystem.class);
        this.magazine = RobotContainer.getSubsystem(Magazine_Subsystem.class);
        this.cmdRPM = target;
    }
    
    public RPMShootCommand(){
       this(defaultShooterRPMs);
    }

    @Override
    public void initialize(){
        table = NetworkTableInstance.getDefault().getTable("ShootCommand");
        shooterState = table.getEntry("ShooterState");

        stage = Stage.DoNothing;
        shooter.off();
    }

    @Override
    public void execute(){
        shooterState.setString(stage.toString());

        switch(stage){
            case DoNothing:
                magazine.driveWheelOff();
                shooter.on(cmdRPM);
                stage = Stage.WaitingForFlyWheel;
            break;

            case WaitingForFlyWheel:
                if(shooter.isReadyToShoot()){
                    magazine.driveWheelOff(); //dont advance indexer if shooting wheels aren't ready
                    stage = Stage.PreparingToShoot;
                }
            break;

            //back the balls away from wheels a touch
            case PreparingToShoot:
                magazine.expellCargo(-0.1);
                backupCounter++;
                if (backupCounter > 20) {
                    backupCounter = 0;
                    stage = Stage.Shooting;
                }                

            case Shooting:
                if(!shooter.isReadyToShoot()){
                    stage = Stage.WaitingForFlyWheel;
                } else magazine.driveWheelOn(1.0);
            break;
        }
    }

    @Override
    public void end(boolean interrupted){
        stage = Stage.DoNothing;
        magazine.driveWheelOff();
        shooter.off();
    }

    public void setFinished(){
        finished = true;
    }
    
    @Override
    public boolean isFinished(){
        return finished;
    }
}
