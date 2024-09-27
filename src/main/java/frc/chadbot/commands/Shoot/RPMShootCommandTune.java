package frc.chadbot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.chadbot.RobotContainer;
import frc.chadbot.Constants.Autonomous;
import frc.chadbot.commands.IntakeCommand;
import frc.chadbot.commands.IntakeCommand.IntakeMode;
import frc.chadbot.subsystems.Intake_Subsystem;
import frc.chadbot.subsystems.Magazine_Subsystem;
import frc.chadbot.subsystems.shooter.FlyWheelRPM;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;
import frc.chadbot.util.PoseMath;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RPMShootCommandTune extends CommandBase{ 
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

    FlyWheelRPM prevRPM;

    private boolean finished = false;

    private double upperP;
    private double upperI;
    private double upperD;

    private double upperFF;
    private double lowerFF;

    private double r_upperP = 0.12;
    private double r_upperI = 0.0;
    private double r_upperD = 4.0;

    private double r_upperFF = 0.034;
    private double r_lowerFF = 0.034;

    private double requestedVelocity = 80;
    private double previousVelocity = 80;
    final String FFID = "Requested upperFF";

    ShooterSettings  cmdSS;         // instance the shooter sees

    final ShooterSettings defaultShooterSettings = new ShooterSettings(requestedVelocity, 0.0, USE_CURRENT_ANGLE, 0.20);

    private VelShootCommand currentShooterCommand;
   // private Pose2d centerField = Constants.Autonomous.hubPose;
    private double distanceToTarget = 0;
    
    public RPMShootCommandTune(double requestedVelocity){
        this.intake = RobotContainer.RC().intake;
        this.shooter = RobotContainer.RC().shooter;
        this.magazine = RobotContainer.RC().magazine;
        ShooterSettings target = new ShooterSettings(requestedVelocity, 0.0, USE_CURRENT_ANGLE, 0.20);
        this.cmdSS = target;
    }
    
    public RPMShootCommandTune(){
        this.intake = RobotContainer.RC().intake;
        this.shooter = RobotContainer.RC().shooter;
        this.magazine = RobotContainer.RC().magazine;
        cmdSS = defaultShooterSettings;
    }

    @Override
    public void initialize(){
        table = NetworkTableInstance.getDefault().getTable("ShootCommand");

        currentShooterCommand = new VelShootCommand(new ShooterSettings(10, 0.0) , 20);
        CommandScheduler.getInstance().schedule(currentShooterCommand);
        RobotContainer.RC().drivetrain.setPose(Autonomous.startPose1);

        SmartDashboard.putNumber("Requested Flywheel P", r_upperP);
        SmartDashboard.putNumber("Requested Flywheel I", r_upperI);
        SmartDashboard.putNumber("Requested Flywheel D", r_upperD);
        SmartDashboard.putNumber("Requested upperFF", r_upperFF);
        SmartDashboard.putNumber("Requested lowerFF", r_lowerFF);

        upperP = shooter.getUpperP();
        upperI = shooter.getUpperI();
        upperD = shooter.getUpperD();
        upperFF = shooter.getUpperF();
        lowerFF = shooter.getLowerF();
    
        SmartDashboard.putNumber("Current Flywheel P", upperP);
        SmartDashboard.putNumber("Current Flywheel I", upperI);
        SmartDashboard.putNumber("Current Flywheel D", upperD);
        SmartDashboard.putNumber("Current upperFF", upperFF);
        SmartDashboard.putNumber("Current lowerFF", lowerFF);

        CommandScheduler.getInstance().schedule(new IntakeCommand((()-> 0.47), ()-> 0.30,  IntakeMode.LoadCargo));
    }

    @Override
    public void execute(){
        checkDashboard();
        getPID();
        checkPID();
        distanceToTarget = PoseMath.poseDistance(RobotContainer.RC().drivetrain.getPose(), Autonomous.hubPose);
    }

    private void getPID(){
        upperP = shooter.getUpperP();
        upperI = shooter.getUpperI();
        upperD = shooter.getUpperD();
        upperFF = shooter.getUpperF();
        lowerFF = shooter.getLowerF();
    
        SmartDashboard.putNumber("Current Flywheel P", upperP);
        SmartDashboard.putNumber("Current Flywheel I", upperI);
        SmartDashboard.putNumber("Current Flywheel D", upperD);
        SmartDashboard.putNumber("Current upperFF", upperFF);
        SmartDashboard.putNumber("Current lowerFF", lowerFF);


        SmartDashboard.putNumber("Velocity Requested", requestedVelocity);
        FlyWheelRPM flyWheelRPM = new FlyWheelRPM();
        shooter.getFlyWheelRPM(flyWheelRPM);
        SmartDashboard.putNumber("Upper RPM", flyWheelRPM.upper);
        SmartDashboard.putNumber("Lower RPM", flyWheelRPM.lower);
        SmartDashboard.putNumber("DistanceToTarget", distanceToTarget);
    }

    private void checkPID(){
        r_upperP = SmartDashboard.getNumber("Requested Flywheel P", upperP);
        r_upperI = SmartDashboard.getNumber("Requested Flywheel I", upperI);
        r_upperD = SmartDashboard.getNumber("Requested Flywheel D", upperD);
        r_upperFF = SmartDashboard.getNumber("Requested upperFF", upperFF);
        r_lowerFF = SmartDashboard.getNumber("Requested lowerFF", lowerFF);

        if (upperP != r_upperP){
            shooter.setPIDUpper(r_upperP, upperI, upperD, upperFF);
            shooter.setPIDLower(r_upperP, upperI, upperD, lowerFF);
        }
        if (upperI != r_upperI){
            shooter.setPIDUpper(upperP, r_upperI, upperD, upperFF);
            shooter.setPIDLower(upperP, r_upperI, upperD, lowerFF);
        }
        if (upperD != r_upperD){
            shooter.setPIDUpper(upperP, upperI, r_upperD, upperFF);
            shooter.setPIDLower(upperP, upperI, r_upperD, lowerFF);
        }
        if (upperFF != r_upperFF){
            shooter.setPIDUpper(upperP, upperI, upperD, r_upperFF);
            shooter.setPIDLower(upperP, upperI, upperD, lowerFF);
        }
        if (lowerFF != r_lowerFF){
            shooter.setPIDUpper(upperP, upperI, upperD, upperFF);
            shooter.setPIDLower(upperP, upperI, upperD, r_lowerFF);
        }
    }

    private void checkDashboard(){
        requestedVelocity = SmartDashboard.getNumber("Velocity Requested", 10);  
        if(requestedVelocity != previousVelocity){
            currentShooterCommand.setFinished();
            currentShooterCommand = new VelShootCommand(requestedVelocity); 
            CommandScheduler.getInstance().schedule(currentShooterCommand);
        }
        previousVelocity = requestedVelocity;
    }

    @Override
    public void end(boolean interrupted){
        currentShooterCommand.setFinished();
        intake.off();
    }

    public void setFinished(){
        finished = true;
    }
    
    @Override
    public boolean isFinished(){
        return finished;
    }


    
}
