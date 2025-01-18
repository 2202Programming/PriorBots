package frc.robot2019.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.RobotMap;
import frc.robot2019.commands.drive.shift.AutomaticGearShiftCommand;

public class GearShifterSubsystem extends SubsystemBase {

    public enum Gear {        
        HIGH_GEAR (DoubleSolenoid.Value.kForward), 
        LOW_GEAR (DoubleSolenoid.Value.kReverse) ;
        private final DoubleSolenoid.Value gearCode;
        Gear(DoubleSolenoid.Value value) { gearCode = value; }
        public DoubleSolenoid.Value solenoidCmd() {return this.gearCode; }
    }

    private long logTimer;
    private boolean isAutoShiftEnabled;
    private final DriveTrainSubsystem driveTrain; //used for vel for autoshift
    
    //State
    Gear curGear = Gear.LOW_GEAR;    // really can grab this from the 
    double shiftPoint;

    public GearShifterSubsystem() {
        logTimer = System.currentTimeMillis();
        driveTrain = RobotContainer.getSubsystem(DriveTrainSubsystem.class);
        addChild("GearShifter", gearShiftSolenoid);
    }

    public GearShifterSubsystem(double _shiftPoint) {
        this();        
        this.shiftPoint = _shiftPoint;
    }

    public void autoshiftEnabled(boolean enable) {
        isAutoShiftEnabled = enable;
    }

    public void log(int interval) {
        if ((logTimer + interval) < System.currentTimeMillis()) { //only post to smartdashboard every interval ms
          logTimer = System.currentTimeMillis();
          SmartDashboard.putBoolean("Autoshift Enabled", isAutoShiftEnabled);  
          SmartDashboard.putString("Gear Shifter State", String.valueOf(getCurGear()));
        }
      }

    //physical devices
    private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(RobotMap.GEARSHIFT_PCM_ID, 
        RobotMap.moduleType, RobotMap.GEARSHIFTUP_SOLENOID_PCM, RobotMap.GEARSHIFTDOWN_SOLENOID_PCM);

    
    public void initDefaultCommand() {
        shiftDown();
        setDefaultCommand(new AutomaticGearShiftCommand());
        isAutoShiftEnabled = true;
    }

    public void shiftUp()  {       
        gearShiftSolenoid.set(Gear.HIGH_GEAR.solenoidCmd());
        curGear = Gear.HIGH_GEAR;
    }

    public void shiftDown() {
        double speed = (driveTrain.getLeftEncoderTalon().getSelectedSensorVelocity()
                + driveTrain.getRightEncoderTalon().getSelectedSensorVelocity()) / 2;
        if ( speed <= shiftPoint) 
        {
            gearShiftSolenoid.set(Gear.LOW_GEAR.solenoidCmd());
            curGear = Gear.LOW_GEAR;
        }
    }

    public Gear getCurGear() {
        return curGear;
    }
}