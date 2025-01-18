package frc.robot2019.input.triggers;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.filter.LinearFilter;  // was .LinearDigitalFilter;
import frc.robot2019.Robot;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
//was a trigger import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj2.command.Command;

public class MotorOverPowerShutdown implements PIDSource, BooleanSupplier {
    final int TAP_MAX = 25;   // this is 0.5 seconds normally
    WPI_TalonSRX motor;
    double powerLimit;
    double avgPower;          //watts
    Command saveMotorCmd;
    LinearFilter movingWindow;

    public MotorOverPowerShutdown(WPI_TalonSRX motor, double powerLimit, double seconds) {
        this.powerLimit = powerLimit;
        this.motor = motor;    
        this.avgPower = 0.0;
        int taps = (int) Math.floor(seconds / Robot.dT);
    
        // build a moving average window
        movingWindow = LinearFilter.movingAverage(taps);
        this.saveMotorCmd = new SaveMotor();

        //install the command and hope it is never used
        this.whenActive(this.saveMotorCmd);

        System.out.println("OverPower " + motor.getDescription() + " watts= " + powerLimit + " - for testing only");
    }

    @Override
    public boolean getAsBoolean() {
        // this is called each frame, so call pidGet() here.
        // Not really a pid, but this is how the filter class works
        pidGet();   //reads values, computes power and saves in the window
        // look for too much average power 
        if (movingWindow.get() > powerLimit) return true;
        return false;
    }

    // monitor power
    double readPower() {
        double oi = motor.getStatorCurrent();   // was getOutputCurrent();
        double ov = motor.getMotorOutputVoltage();
        return Math.abs(oi*ov);
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        //dpl - don't think this matters
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return  PIDSourceType.kDisplacement;    //dpl should not matter for our use as a filter
    }

    // inserts value into the filter, called by filter.
    @Override
    public double pidGet() {
        return readPower();  //value used for filter
	}

    // SaveMotor will disable the motor and set speed to zero of the overpower triggers
    class SaveMotor extends Command {
        SaveMotor() {           
        }

        @Override
        public void execute() {
            motor.set(0.0);
            motor.disable();

            //Make noise
            System.out.println("****MOTOR POWER TRIGGERED**** -->" + motor.getName() );
        }
        
        // keep in the safe state, this command will have to get kicked out.
        @Override
        public boolean isFinished() { 
            return false; 
        }
    }

}