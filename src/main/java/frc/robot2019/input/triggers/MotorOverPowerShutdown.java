package frc.robot2019.input.triggers;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.filter.LinearFilter;  // was .LinearDigitalFilter;

//was a trigger import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot2019.Constants;


//WIP - needs testing to see how it interacts with other commands
public class MotorOverPowerShutdown  implements  BooleanSupplier  {
    final int TAP_MAX = 25;   // this is 0.5 seconds normally (25 taps * .02 sec/sample)
    WPI_TalonSRX motor;       // motor to watch
    double powerLimit;
    double avgPower;          //watts
    Command saveMotorCmd;
    LinearFilter movingWindow;
    Trigger trigger;

    public MotorOverPowerShutdown(WPI_TalonSRX motor, double powerLimit, double seconds) {
        this.powerLimit = powerLimit;
        this.motor = motor;    
        this.avgPower = 0.0;
        int taps = (int) Math.floor(seconds / Constants.dT);
    
        // build a moving average window
        movingWindow = LinearFilter.movingAverage(taps);
        //Create the trigger based on this object's BooleanSupplier which
        trigger = new Trigger(this);
        //install the command and hope it is never used
        trigger.onTrue(new SaveMotor());

        System.out.println("OverPower " + motor.getDescription() + " watts= " + powerLimit + " - for testing only");
    }

    @Override
    public boolean getAsBoolean() { 
        // look for too much average power 
        double avg_pwr = movingWindow.calculate(readPower());
        return avg_pwr > powerLimit;
    }

    // monitor power
    double readPower() {
        double i = motor.getStatorCurrent();   // was getOutputCurrent();
        double v = motor.getMotorOutputVoltage();
        return Math.abs(i*v);
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
            System.out.println("****MOTOR POWER TRIGGERED**** -->" + motor.getDescription() );
        }
        
        // keep in the safe state, this command will have to get kicked out.
        @Override
        public boolean isFinished() { 
            return false; 
        }
    }

}