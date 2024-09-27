package frc.chadbot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.chadbot.RobotContainer;
import frc.chadbot.subsystems.Intake_Subsystem;
import frc.chadbot.subsystems.Magazine_Subsystem;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class BasicShootCommand extends CommandBase{ 
    public static final double USE_CURRENT_ANGLE = 0.0;

    final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    final Shooter_Subsystem shooter;
    final double TESTANGLE = 0.0;
    final double TESTTOL = 0.02;
    final int BackupPeriod;

    int ballCount = 999;
    int backupCounter = 0;

    NetworkTable table;
    NetworkTableEntry ntUpperRPM;   //FW speeds (output)
    NetworkTableEntry ntLowerRPM;
    NetworkTableEntry ntBallVel;    // ball physics (input) 
    NetworkTableEntry ntBallRPS;
    NetworkTableEntry shooterState;

    ShooterSettings specialSettings;
    
    ShooterSettings  cmdSS;         // instance the shooter sees
    ShooterSettings  prevSS;        // instance for prev State

    private boolean finished = false;

    final static ShooterSettings defaultShooterSettings = new ShooterSettings(20.0, 0.0, USE_CURRENT_ANGLE, 0.01);

    public enum Stage{
        DoNothing("Do Nothing"),
        WaitingForFlyWheel("Waiting for flywheel"),
        BackingMagazine("Backing  Mag"),
        PreparingToShoot("Preparing to Shoot"),
        WaitingForSolution("doing complex math"),
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
    
    public BasicShootCommand(ShooterSettings shooterSettings, int backupFrameCount){
        this.intake = RobotContainer.RC().intake;
        this.shooter = RobotContainer.RC().shooter;
        this.magazine = RobotContainer.RC().magazine;
        specialSettings = shooterSettings;
        BackupPeriod = backupFrameCount;  //number of frames to move mag back slowly 5-20
        addRequirements(magazine,shooter);
    }

    @Override
    public void initialize(){
        table = NetworkTableInstance.getDefault().getTable("ShootCommand");
        ntBallVel = table.getEntry("BallVel");
        ntBallRPS = table.getEntry("BallRPS");
        shooterState = table.getEntry("ShooterState");

        ntBallVel.setDouble(0);
        ntBallRPS.setDouble(0);

        cmdSS = new ShooterSettings(); //defaultShooterSettings SUS 
        prevSS = new ShooterSettings(cmdSS);

        stage = Stage.DoNothing;
        shooter.off();
        magazine.driveWheelOff();
    }

    @Override
    public void execute(){
        shooterState.setString(stage.toString());

        cmdSS = specialSettings;

        switch(stage){
            case DoNothing:
                backupCounter = 0;
                stage = Stage.BackingMagazine;
                magazine.expellCargo(0.1);
            break;

            case BackingMagazine:                
                backupCounter++;
                if (backupCounter > BackupPeriod) {
                    backupCounter = 0;
                    magazine.driveWheelOff();
                    stage = Stage.WaitingForFlyWheel;
                    shooter.spinup(cmdSS);
                }                
            break;

            case WaitingForFlyWheel:
                if(shooter.isReadyToShoot()){
                    stage = Stage.WaitingForSolution;
                }
            break;

            case WaitingForSolution:
                // do fancy check and when ready, goto shooting
                stage = Stage.Shooting;
                magazine.driveWheelOn(1.0);
                break;

            case Shooting:
                if(!shooter.isReadyToShoot()){
                    magazine.driveWheelOff();
                    stage = Stage.WaitingForFlyWheel;
                }
            break;
            default:
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
