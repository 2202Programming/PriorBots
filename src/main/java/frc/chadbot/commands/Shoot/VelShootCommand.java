package frc.chadbot.commands.Shoot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.chadbot.Constants.Autonomous;
import frc.chadbot.Constants.Shooter;
import frc.chadbot.subsystems.Intake_Subsystem;
import frc.chadbot.subsystems.Magazine_Subsystem;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.Limelight;
import frc.lib2202.subsystem.LimelightHelpers;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.util.PoseMath;

/**
 * Use the gated version - it ties into the light gates on the magazine
 */
public class VelShootCommand extends Command implements SolutionProvider{ 

    public static final double USE_CURRENT_ANGLE = 0.0;

    final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    final Shooter_Subsystem shooter;
    final SolutionProvider solutionProvider;
    final Limelight limelight;
    final SwerveDrivetrain drivetrain;
      
    final double TESTANGLE = 0.0;
    final double TESTTOL = 0.02;
    final int BackupPeriod;

    //how long to wait for a solution before shooting without it
    final int maxSolutionWait = 1000;

    int ballCount = 999;
    int backupCounter = 0;
    int solutionTimer = 0;
    double currentDistance = 0;

    NetworkTable table;
    NetworkTable drivetrainTable;
    NetworkTableEntry ntUpperRPM;   //FW speeds (output)
    NetworkTableEntry ntLowerRPM;
    NetworkTableEntry ntBallVel;    // ball physics (input) 
    NetworkTableEntry shooterState;
    NetworkTableEntry distance;
    NetworkTableEntry NToutOfRange;
    final String NT_Name = "Shooter"; 


    ShooterSettings m_shooterSettings;
    ShooterSettings  cmdSS;         // instance the shooter sees
    
    double calculatedVel = 20;
    double distanceOffeset = 0.0;

    boolean finished = false;
    //private boolean solution = true;
    boolean outOfRange = false;
    boolean autoVelocity = true;

    double log_counter = 0;

    //for velocity calculations
    //cut over distance between two distance/speed linear relationships
    final double FARDISTANCE = 4.8;

    //close slope/intercept.  Slope will change multiplier between distance and RPM.  Intercept will add RPMs to all distances equally.
    final double SLOPE = 4.872;
    final double INTERCEPT = 26.8;

    //change slope multiplier to increase FPS at far distances.
    final double FARSLOPE = SLOPE*1.4;
    final double FARINTERCEPT =  FARDISTANCE * SLOPE + INTERCEPT;

  

    final static ShooterSettings defaultShooterSettings = new ShooterSettings(20.0, 0.0, USE_CURRENT_ANGLE, 0.01);

    public enum Stage{
        DoNothing("Do Nothing"),
        WaitingForFlyWheel("Waiting for flywheel"),
        BackingMagazine("Backing  Mag"),
        PreparingToShoot("Preparing to Shoot"),
        WaitingForSolution("Waiting for Solution"),
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
    
    public VelShootCommand(ShooterSettings shooterSettings, int backupFrameCount, SolutionProvider solutionProvider){
        this.intake = RobotContainer.getSubsystem(Intake_Subsystem.class);
        this.shooter = RobotContainer.getSubsystem(Shooter_Subsystem.class);
        this.magazine = RobotContainer.getSubsystem(Magazine_Subsystem.class);
        this.limelight = RobotContainer.getSubsystem(Limelight.class);
        this.drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);

        // the default solution provider is always true
        this.solutionProvider = (solutionProvider ==null) ? this : solutionProvider;
        m_shooterSettings = shooterSettings;
        BackupPeriod = backupFrameCount;  //number of frames to move mag back slowly 5-20
        addRequirements(magazine,shooter);

        table = NetworkTableInstance.getDefault().getTable(NT_Name);

        ntBallVel = table.getEntry("/VelShootCmd/BallVel");
        shooterState = table.getEntry("/VelShootCmd/ShooterState");
        distance = table.getEntry("/VelShootCmd/Distance");
        NToutOfRange = table.getEntry("/VelShootCmd/OutOfRange");
    }

    public VelShootCommand(ShooterSettings shooterSettings, int backupFrameCount)
    {
        this(shooterSettings, backupFrameCount, null);
    }

    public VelShootCommand(double requestedVelocity){  //velocity only overload
        this(new ShooterSettings(requestedVelocity, 0.0, 0.0, 0.1), 20, null);
    }

    //overload constructor to allow for shooting with autovelocity RPM adjustment off (defaults to true in other constructors)
    public VelShootCommand(boolean autoVelocity){
        this(defaultShooterSettings, 20, null);
        this.autoVelocity = autoVelocity;        
    }

    //overload constructor to allow for shooting with autovelocity RPM adjustment off
    public VelShootCommand(double requestedVelocity, boolean autoVelocity){
        this(new ShooterSettings(requestedVelocity, 0.0, 0.0, 0.1), 20, null);
        this.autoVelocity = autoVelocity;        
    }

    public VelShootCommand()
    {
        this(defaultShooterSettings, 20, null);
    }


    @Override
    public void initialize(){
        distanceOffeset = 0;
        cmdSS = m_shooterSettings; 
        stage = Stage.DoNothing;
        shooter.off();
        magazine.driveWheelOff();
    }

    @Override
    public void execute(){
        NTupdates();
        calculateDistance();
        calculateVelocity();
        
        //if autovelocity is true will calculate a new RPM speed based on the distance
        //otherwise RPMs should be constant based on the constructor parameters
        if (autoVelocity) {
            if((calculatedVel) != cmdSS.vel){
                cmdSS.vel = calculatedVel;   // = new ShooterSettings(calculatedVel, 0);
                shooter.spinup(cmdSS);
            }
        } 

        switch(stage){
            case DoNothing:
                backupCounter = 0;
                stage = Stage.BackingMagazine;
                magazine.expellCargo(0.1);
            break;

            case BackingMagazine:                
                backupCounter++;
                if (backupCounter > BackupPeriod) {
                    // issues commands for next stage 
                    stage = Stage.WaitingForFlyWheel;
                    backupCounter = 0;
                    magazine.driveWheelOff();           // balls are off the flywheels
                    intake.off();
                    shooter.spinup(cmdSS);              // spin shooter up
                }                
            break;

            case WaitingForFlyWheel:
                if (shooter.isReadyToShoot()) {
                    stage = Stage.WaitingForSolution;
                }
            break;

            case WaitingForSolution:
                solutionTimer++;
                if (solutionProvider.isOnTarget() || (solutionTimer > maxSolutionWait)) {
                    stage = Stage.Shooting;
                    magazine.driveWheelOn(1.0);
                    intake.on(0.0, 0.5);
                    solutionTimer = 0;
                }
                break;

            case Shooting:
                if (!shooter.isReadyToShoot()){
                    magazine.driveWheelOff();
                    intake.off();
                    shooter.spinup(cmdSS); //in case a new velocity has been set due to a new distance
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
        intake.off();
        shooter.off();
    }

    public void setFinished(){
        finished = true;
    }
    
    @Override
    public boolean isFinished(){
        return finished;
    }

    public void calculateDistance(){
        currentDistance = PoseMath.poseDistance(drivetrain.getPose(), Autonomous.hubPose); //crappy estimate from odometery
        if (limelight.getTarget() && limelight.getLEDStatus()){
            //calculate current distance with limelight area instead of odometery, use our ll's name to get from NT.
            currentDistance = estimateDistance(limelight.getName()); 
        }
        currentDistance += distanceOffeset;  //add in velocity based distance offset
    }

    public void calculateVelocity(){    
        double m_slope = SLOPE;
        double m_intercept = INTERCEPT;  

        if (currentDistance > FARDISTANCE){
            m_slope = FARSLOPE;
            m_intercept = FARINTERCEPT;
            calculatedVel = m_slope*(currentDistance-FARDISTANCE) + m_intercept; //distnce vs. velocity trendline for long range positioner
        } else {
            calculatedVel = m_slope*currentDistance + m_intercept; //distnce vs. velocity trendline for long range positioner
        }
         
        if (calculatedVel > Shooter.kMaxFPS){
            outOfRange = true;
            calculatedVel = Shooter.kMaxFPS; //don't ask shooter to go above max FPS otherwise can get stuck waiting for impossible goals
        } else {
            outOfRange = false;
        }
    }

    public double getCalculatedVel(){
        return calculatedVel;
    }

    public void setCalculatedVel(double velocity){
        calculatedVel = velocity;
    }

    public double getDistanceOffset(){
        return distanceOffeset;
    }

    //since currentDistance is constantly recalculated, use a 2nd variable for the offset due to robot motion which comes in from the DriveController.
    public void setdistanceOffset(double offset){
        distanceOffeset = offset;
    }

    private void NTupdates(){
        log_counter++;
        if ((log_counter%20)==0) {
            ntBallVel.setDouble(calculatedVel);
            shooterState.setString(stage.toString());
            distance.setDouble(currentDistance);
            NToutOfRange.setBoolean(outOfRange);
        }
    }

  public static double estimateDistance(String llname) {
    //TODO - verify this works, it should reach into the NT and get the latest TY set by LL
    // might need to make sure we are on retro pipeline.  @Jason - can you review...  1/16/2025
    double ty = LimelightHelpers.getTY(llname);
    double targetOffsetAngle_Vertical = ty;  // was ty.getDouble(0.0);
    // how many degrees back is your limelight rotated from perfectly vertical?
    // both because why not (and that's what the copy-pasta had)
    double angleToGoalDegrees = Shooter.LL_MOUNT_ANGLE_DEG + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    // calculate distance
    return (((Shooter.GOAL_HEIGHT_TO_FLOOR_INCHES - Shooter.LL_LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians)
        + Shooter.EDGE_TO_CENTER_INCHES) / Shooter.METERS_TO_INCHES);
  }

}
