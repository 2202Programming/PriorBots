package frc.robot2024.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Intake;
import frc.robot2024.subsystems.Transfer;

/*
 * This is used if the intakeSequence is interrupted but we had a note
 * in flight.  It finishes the count and stops the rollers.
 *
 */

public class FinishIntakeSequence extends Command {
    final Intake intake;
    final Transfer transfer;
    //use DONE_COUNT defined in IntakeSequence
    final int DONE_COUNT = IntakeSequence.DONE_COUNT; 
    int count;
    boolean stay_down;
    boolean transfer_detected;

    /**
     * 
     * @param count - where were were when interrupted
     * @param stay_down
     */
    public FinishIntakeSequence(int count, boolean stay_down, boolean transfer_detected){
        this.count = count;
        this.stay_down = stay_down;
        this.transfer_detected = transfer_detected;
        this.intake = RobotContainer.getSubsystem(Intake.class);
        this.transfer = RobotContainer.getSubsystem(Transfer.class);
    }

    @Override
    public void initialize() {
        // put the rollers back on
        // System.out.println("RUNNING FINISH SEQUENCE");
        intake.setIntakeSpeed(Intake.RollerMaxSpeed); // [cm/s]
        transfer.setSpeed(50.0);  //[cm/s]
    }


    @Override
    public boolean isFinished() {
        // run until count expires
        if(transfer_detected){
            // System.out.println("HIT TRANSFER IN FINISH");
        if (++count >= DONE_COUNT) {
            // stop at hopefully the same location
            intake.setMaxVelocity(Intake.TravelUp);
            transfer.setSpeed(0.0);
            intake.setIntakeSpeed(0.0);

            //handle stay_down flag from original IntakeSequence
            if (!stay_down) {
                intake.setMaxVelocity(Intake.TravelUp);
                intake.setAngleSetpoint(Intake.UpPos);
            }
            return true;
        }
    }
    else if (!transfer_detected){
        transfer_detected = transfer.senseNote();
    }
        return false;

    }

}
