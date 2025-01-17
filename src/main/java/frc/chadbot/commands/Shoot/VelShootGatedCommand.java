package frc.chadbot.commands.Shoot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.chadbot.commands.MagazineController;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;


public class VelShootGatedCommand extends VelShootCommand{ 

    final MagazineController mag_ctrl;
  
    //use own network tables
    final String NT_Name = "ShooterGated";
    final NetworkTableEntry nte_BallVel;        
    final NetworkTableEntry nte_shooterState;
    final NetworkTableEntry nte_distance;
    final NetworkTableEntry nte_outOfRange;
     
    //Gated Stages are different.
    enum GStage{
        WaitingForFlyWheel("Waiting for flywheel"),
        WaitingOnMag("MagCtrl Not ready"),
        PreparingToShoot("Preparing to Shoot"),
        WaitingForSolution("Waiting for Solution"),
        Shooting("Shooting");

        String name;

        private GStage(String name){
            this.name = name;
        }

        public String toString(){
            return name;
        }
    }
    
    GStage stage;

    /**
     * VelShootGatedCommand - uses magazine with lightgates. Details of magazine ball management is
     * completely inside the magazineController. 
     * 
     * @param shooterSettings       container for shooter speeds/angle
     * @param mag_ctrl              interface for controlling magazine
     * @param solutionProvider      interface for target tracking 
     * @param autoVelocity          calculates and controls shooter velocity based on distance
     */

    public VelShootGatedCommand(ShooterSettings shooterSettings, MagazineController mag_ctrl, SolutionProvider solutionProvider, boolean autoVelocity){
        super(shooterSettings, 0, solutionProvider);
        this.mag_ctrl = mag_ctrl;
        this.autoVelocity = autoVelocity;

        cmdSS = new ShooterSettings(m_shooterSettings); 

        getRequirements().remove(magazine);   //using mag_ctrl, not magazine directly

        NetworkTable table = NetworkTableInstance.getDefault().getTable(NT_Name);
        nte_BallVel = table.getEntry("/VelGateShootCmd/BallVel");
        nte_shooterState = table.getEntry("/VelGateShootCmd/ShooterState");
        nte_distance = table.getEntry("/VelGateShootCmd/Distance");
        nte_outOfRange = table.getEntry("/VelGateShootCmd/OutOfRange");
    }

    public VelShootGatedCommand(ShooterSettings shooterSettings, MagazineController mag_ctrl)
    {
        this(shooterSettings, mag_ctrl, null, false);
    }

    public VelShootGatedCommand(double requestedVelocity, MagazineController mag_ctrl)
    {
        this(new ShooterSettings(requestedVelocity, 0.0, 0.0, 0.1), mag_ctrl, null, false);
    }

    public VelShootGatedCommand(MagazineController mag_ctrl, SolutionProvider solutionProvider)
    {
        //CHANGE HERE FOR SPIN ADJUSTMENT
        this(new ShooterSettings(20, -5.0, 0.0, 0.1), mag_ctrl, solutionProvider, true); //negative is front spin
    }



    @Override
    public void initialize(){
        stage = GStage.WaitingOnMag;
        // should be off already shooter.off();
    }

    @Override
    public void execute(){
        // thes methods have side-effect flags
        calculateDistance();  // sets currentDistance
        calculateVelocity();  // sets outOfRange and calculatedVelocity

        //if autovelocity is true will calculate a new RPM speed based on the distance and adjust positioner
        //otherwise RPMs should be constant based on the constructor parameters
        if (autoVelocity) {
            if (mag_ctrl.safeToSpinUp() && (calculatedVel != cmdSS.vel)) {
                cmdSS.vel = calculatedVel ;   //shouldn't have to create new object, just change vel
                shooter.spinup(cmdSS);
            }
        } 

        switch(stage){
            case WaitingOnMag:                
                if (mag_ctrl.safeToSpinUp()) {
                    stage = GStage.WaitingForFlyWheel;
                    intake.off();
                    shooter.spinup(cmdSS);              // spin shooter up
                }                
            break;

            case WaitingForFlyWheel:
                if (shooter.isReadyToShoot()) {
                    stage = GStage.WaitingForSolution;
                }
            break;

            case WaitingForSolution:
                if (solutionProvider.isOnTarget()) {
                    stage = GStage.Shooting;
                    mag_ctrl.feederOn();
                    intake.on(0.0, 0.2);
                }
                break;

            case Shooting:
                if (!shooter.isReadyToShoot()){
                    mag_ctrl.feederOff();
                    intake.off();
                    shooter.spinup(cmdSS); //in case a new velocity has been set due to a new distance
                    stage = GStage.WaitingForFlyWheel;
                }
            break;
            default:
                break;
        }

        NTupdates();
    }

    @Override
    public void end(boolean interrupted){
        mag_ctrl.feederOff();
        intake.off();
        shooter.off();
    }

    @Override
    public boolean isFinished() {
        return (mag_ctrl.getBallCount() <= 0);
    }

    void NTupdates(){
        log_counter++;
        if ((log_counter%10)==0) {
            nte_BallVel.setDouble(calculatedVel);
            nte_shooterState.setString(stage.toString());
            nte_distance.setDouble(currentDistance);
            nte_outOfRange.setBoolean(outOfRange);
        }
    }
}
