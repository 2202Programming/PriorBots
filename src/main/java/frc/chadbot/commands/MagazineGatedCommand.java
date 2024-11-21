package frc.chadbot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.chadbot.RobotContainer;
import frc.chadbot.commands.MovePositioner.PositionerMode;
import frc.chadbot.subsystems.Intake_Subsystem;
import frc.chadbot.subsystems.Magazine_Subsystem;
import frc.chadbot.subsystems.ifx.DriverControls.Id;

/**
 * 
 * Upper LG is triggered when a ball is near the flywheels but
 * not close to touching them. There should be a bit of rotation left
 * to move to flywheel and back up.
 * 
 * Lower LG is triggered when the ball just hits the base of the magazine.
 * Sometimes the ball will rest on base and not trigger the gate (maybe move the
 * gate?).
 * 
 * It is possible to move one ball from the lower lg to a non-gated zone
 * where the upper lg and lower lg are both off. This must be avoided for the
 * starting position.
 * 
 * External Driver Events:
 * Eject
 * FeedForShooting
 * 
 * Goals:
 * report count
 * keep balls off flywheels
 * simplify driver workload
 * 
 */
public class MagazineGatedCommand extends Command implements MagazineController {
    final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    final double magazineSpeed;

    final int MovingTwoInFC = 10; // moves forward to get 2nd ball into position
    final int SafetyBackupFC = 6; // back off the flywheels
    final int ConfirmEmptyFC = 8; // moves backward to check blind spot
    final int AlignFC = 10; // moves forward to align when both gates have a ball
    final int SideOffDelayFC = 50;

    final double SlowRotate = 0.5;

    boolean prev_deployed;
    boolean prev_lower_lg;
    boolean prev_upper_lg;
    int frame_count_down; // used for open loop position, stop on zero.
    int side_off_count_down;

    int ball_count;
    int last_ball_count;

    public enum MagazineState {
        ConfirmEmpty("confirming empty"), // ball in deadzone?
        Empty("empty"),
        OneBall_Lower("1-ball-lower"),
        MovingToUpper("move 1-ball to upper"),
        OneBall_Upper("one ball in upper"),
        MovingBallTwoIn("moving 2-ball in"),
        BackingUp("backing up for safety"),
        AlignTwoBalls("align 2-ball"),
        TwoBalls("two balls ready");

        String name;

        private MagazineState(String name) {
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }

    boolean spinup_safe = false;
    boolean feed_request = false; // external event
    boolean eject_request = false; // external event
    final double SideSpeed = 0.5; // positive is loading
    final double IntakeSpeed = 0.6;
    boolean sides_on = false; // used for countdown delay on retract
    double side_speed = 0.0; // side roller speed being commanded for eject/feed states

    MagazineState state = MagazineState.Empty;
    MagazineState prev_state = state;

    //final Command ejectCmd;
    //final Command feedCmd;
    final Command movePositionerHigh = new MovePositioner(PositionerMode.High);

    // Network reporting
    final String NT_Name = "/Magazine-Controller";
    final NetworkTableEntry nte_lowergate;
    final NetworkTableEntry nte_uppergate;
    final NetworkTableEntry nte_feed_request;
    final NetworkTableEntry nte_eject_request;
    final NetworkTableEntry nte_spinup_safe;
    final NetworkTableEntry nte_state;
    final NetworkTableEntry nte_ballcount;
    final NetworkTableEntry nte_sides;

    // Constructor
    public MagazineGatedCommand(double magazineSpeed) {
        this.magazine = RobotContainer.RC().magazine; // just get the magazine from RC
        this.intake = RobotContainer.RC().intake;
        this.magazineSpeed = magazineSpeed;

        //ejectCmd = new EjectCmd(this);
        //feedCmd = new FeedCmd(this);

        // setup all the tables for debugging
        NetworkTable table = NetworkTableInstance.getDefault().getTable(NT_Name);
        nte_lowergate = table.getEntry("/lowerGate");
        nte_uppergate = table.getEntry("/upperGate");
        nte_feed_request = table.getEntry("/feederOn");
        nte_eject_request = table.getEntry("/ejectorOn");
        nte_spinup_safe = table.getEntry("/spinupSafe");
        nte_state = table.getEntry("/state");
        nte_ballcount = table.getEntry("/ballCount");
        nte_sides = table.getEntry("/sidesOn");

        prev_deployed = intake.isDeployed();

        side_speed = SideSpeed;
        addRequirements(magazine); // required for a default command
    }

    // Accessor for supporting eject/feed commands
    public Command getEjectCmd() {
        return new EjectCmd(this);
    }

    public Command getFeedCmd() {
        return new FeedCmd(this);
     }

    /**
     * Called on scheduling or whenever we need to figure out the state of the
     * cargo we are carrying, like after feed or eject request.
     */
    public void initialize() {
        intake.off();           //everything off at start or eject/feed off
        sides_on = false;
        spinup_safe = false;
        side_speed = SideSpeed;
        // read gates, save as previous values for edge detection
        prev_upper_lg = magazine.upperGateBlocked();
        prev_lower_lg = magazine.lowerGateBlocked();
        last_ball_count = 0;

        // sort out gates and ball positions for state machine
        state = (prev_lower_lg) ? MagazineState.OneBall_Lower : MagazineState.Empty;
        if (prev_upper_lg) {
            state = (state == MagazineState.Empty) ? MagazineState.OneBall_Upper : MagazineState.AlignTwoBalls;
        }
        // if we didn't find anything, still need to check the blind zone
        state = (state == MagazineState.Empty) ? MagazineState.ConfirmEmpty : state;

        if (state == MagazineState.ConfirmEmpty) {
            frame_count_down = ConfirmEmptyFC;
        }
        if (state == MagazineState.AlignTwoBalls) {
            frame_count_down = AlignFC;
        }
    }

    public void execute() {
        boolean lower_lg = magazine.lowerGateBlocked();
        boolean upper_lg = magazine.upperGateBlocked();
        rumbleMag();
        
        // handle driver requests or execute magazine state machine
        if (feed_request) {
            spinup_safe = true;
            magazine.driveWheelOn(magazineSpeed);
            checkBallCount(upper_lg);
            sides_on = true;
            side_speed = SideSpeed;

        } else if (eject_request) {
            spinup_safe = false;
            magazine.driveWheelOn(-magazineSpeed);
            sides_on = true;
            side_speed = -SideSpeed;
            intake.on(-IntakeSpeed, -SideSpeed); // ejecting, run everything reversed, turn off in init()

        } else {
            // Run the normal handler
            sidesOnCheck();     //based on intake being deployed or not, with delay
            switch (state) {
                case ConfirmEmpty:
                    // back up for a bit to make sure we don't have a ball
                    sides_on = false;
                    magazine.driveWheelOn(-magazineSpeed);
                    if (--frame_count_down <= 0 || lower_lg) {
                        magazine.driveWheelOff();
                        // goto empty either way, lg will move to next state
                        state = MagazineState.Empty;
                    }
                    spinup_safe = false;
                    break;

                case Empty:
                    ball_count = 0;
                    // Looking for ball to trigger lower gate,
                    if (lower_lg && prev_lower_lg) {
                        state = MagazineState.OneBall_Lower;
                    }
                    spinup_safe = false;
                    break;

                case OneBall_Lower:
                    ball_count = 1;
                    spinup_safe = true;
                    if (lower_lg) {
                        state = MagazineState.MovingToUpper;
                        magazine.driveWheelOn(magazineSpeed * SlowRotate);
                    } else {
                        // must have bounced out???
                        state = MagazineState.ConfirmEmpty;
                    }
                    break;

                case MovingToUpper:
                    spinup_safe = true;
                    // Run the mag until the ball hits the upper limit, then next state
                    // run mag for 1 extra frame so prev and current match
                    if (upper_lg) {
                        magazine.driveWheelOff();
                        state = MagazineState.OneBall_Upper;
                    }
                    break;

                case OneBall_Upper: 
                    ball_count = 1;   // could state here
                    spinup_safe = true;
                    // Now we are looking for the 2nd ball to be trigger
                    if (lower_lg & prev_lower_lg) {
                        spinup_safe = false;
                        magazine.driveWheelOn(magazineSpeed);
                        state = MagazineState.MovingBallTwoIn;
                        frame_count_down = MovingTwoInFC;
                    }
                    break;

                case MovingBallTwoIn:
                    // moving second ball into magazine
                    ball_count = 2;
                    sides_on = false;
                    spinup_safe = false;
                    if (--frame_count_down <= 0) {
                        magazine.driveWheelOff();
                        state = MagazineState.BackingUp;
                        frame_count_down = SafetyBackupFC;
                        magazine.driveWheelOn(-magazineSpeed);
                    }
                    break;

                case BackingUp:
                    sides_on = false;
                    spinup_safe = false;
                    if (--frame_count_down <= 0) {
                        magazine.driveWheelOff();
                        intake.retract();     //only on edge, driver can redeploy
                        intake.off();
                        state = MagazineState.TwoBalls;
                        sides_on = false;
                    }
                    break;

                case AlignTwoBalls:
                    // move balls up to flywheel
                    sides_on = false;
                    spinup_safe = false;
                    magazine.driveWheelOn(magazineSpeed * SlowRotate);
                    if (--frame_count_down <= 0) {
                        // done moving forward, next backup
                        magazine.driveWheelOn(-magazineSpeed);
                        frame_count_down = SafetyBackupFC;
                        state = MagazineState.BackingUp;
                    }
                    break;

                case TwoBalls:
                    spinup_safe = true;
                    break;

                default:
                    break;
            }
        }
        
        if (sides_on) {
            intake.sidesOn(side_speed);
        } else {
            intake.sidesOff();
        }

        // save prev
        prev_lower_lg = lower_lg;
        prev_upper_lg = upper_lg;
        prev_state = state;
        updateNT();
    }

    void checkBallCount(boolean current_upper) {
        // look for high --> low edge to detect ball leaving upper gate
        if ((prev_upper_lg == true) && (current_upper == false)) {
            // ball shot
            ball_count--;
        }
    }

    void sidesOnCheck() {
        // sides on, only if deployed.
        boolean deployed = intake.isDeployed();
        if (deployed) {
            sides_on = true;
            side_off_count_down = SideOffDelayFC;
        }

        if (--side_off_count_down <= 0) {
            sides_on = false;
            side_off_count_down = 0;
        }
        prev_deployed = deployed;
    }

    // External Event Handling
    public void feederOn() {
        feed_request = true;
        eject_request = false;
    }

    public void feederOff() {
        feed_request = false;
        initialize(); // redo gates and shutoff feeder
    }

    public void ejectOn() {
        eject_request = true;
        feed_request = false;
    }

    public void ejectOff() {
        eject_request = false;
        initialize(); // read gates and shutoff feeder
    }

    // Safe when we have cargo and are not on the flywheels
    public boolean safeToSpinUp() {
        return spinup_safe;
    }

    public int getBallCount() {
        return ball_count;
    }

    // This command never really ends, it always runs to manage the cargo
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        magazine.driveWheelOff();
    }

    void updateNT() {
        nte_lowergate.setBoolean(prev_lower_lg);
        nte_uppergate.setBoolean(prev_upper_lg);
        nte_feed_request.setBoolean(feed_request);
        nte_eject_request.setBoolean(eject_request);
        nte_spinup_safe.setBoolean(spinup_safe);
        nte_ballcount.setDouble(ball_count);
        nte_state.setString(state.toString());
        nte_sides.setBoolean(sides_on);
    }

    /**
     * Default on/off commands for Driver Events.
     */
    class EjectCmd extends Command {
        MagazineController controller;

        EjectCmd(MagazineController c) {
            controller = c;
            setName("EjectCommand");
        }

        @Override
        public void initialize() {
            controller.ejectOn();
        }

        @Override
        public void end(boolean interrupted) {
            controller.ejectOff();
        }
    }

    class FeedCmd extends Command {
        MagazineController controller;

        FeedCmd(MagazineController c) {
            controller = c;
            setName("FeedCommand");
        }

        @Override
        public void initialize() {
            controller.feederOn();
        }

        @Override
        public void end(boolean interrupted) {
            controller.feederOff();
        }

    }
    //rumble controllers if magazine is full
    void rumbleMag(){
        if (ball_count==2 && last_ball_count==1){ //turn on rumble for double ball notification
            CommandScheduler.getInstance().schedule(new JoystickRumble(Id.Driver, 1, 2));
            CommandScheduler.getInstance().schedule(new JoystickRumble(Id.Assistant, 1, 2));
        }
        if (ball_count==1 && last_ball_count==0){ //turn on rumble for single ball notification
            CommandScheduler.getInstance().schedule(new JoystickRumble(Id.Driver, 0.5, 1));
            CommandScheduler.getInstance().schedule(new JoystickRumble(Id.Assistant, 0.5, 1));
        }
        last_ball_count = ball_count;
    }
}
