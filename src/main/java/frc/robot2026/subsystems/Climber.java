package frc.robot2026.subsystems;
// Copyright (c) FIRST and other WPILib contributors.

import com.revrobotics.spark.SparkFlex;

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;
import frc.robot2026.Constants.CAN;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */
    public final static double PowerUpPosition = 0.0; // [cm]
    public final static double ClimbPositon = 7.0;  // [cm]  go here when climbing
    public final static double ExtendPosition = 22.2; // [cm]  Was reading 28
    public final static double ClimbCalibrateVel = 2.0; // [cm/s]

    final double GearRatio = 1.0 / 25.0;
    final double R_pully = 1.0;  //[cm] radius of pully
    double conversionFactor = 2.0 * Math.PI * R_pully * GearRatio;  // Circumfrance of pulley * gear ratio
    final double maxVel = 100.0; // placeholder. [cm/s]
    final double maxAccel = 10.0; // placevholder [cm/s^2]
    double posTol = 0.25; // [cm]
    double velTol = .50; // [cm/s]
    final int STALL_CURRENT = 80; // [Amp] placeholder 
    final int FREE_CURRENT = 5;   // [Amp] placeholder  
    
    double climbingPos = 21; //[cm]

    public final Arm l_arm;
    public final Arm r_arm; 
    /* My logic for making these public is to allow access to the individual methods while outside the system.
       Otherwise that will just mean making more methods, and that seems like a waste - Gavin 
        Could be, normally SubSystem API should eliminate the need. We will see. Class needs to be public too 
        though to allow method access. - Mr.L
    */
    public class Arm implements Sendable {
        final NeoServo servo;
        String name;
     
        // each arm needs own copy of pids, especially the softare position pid which is run by servo.periodic()
        PIDController posPID = new PIDController(4.0, 0.0015, 0.125); //TODO These values are speculative, they couldnt be properly measured due to conversion error. Fix -G
        PIDFController hwVelPID = new PIDFController(0.02, 0.00015, 0, 0.0285); // Little bit of bouncing, could do better

        Arm(int CANID, String side, boolean inverted, String name) {            
            this.name = name;
            hwVelPID.setIZone(10.0); // TODO record typical climb, set to just over typical error
            hwVelPID.setIntegratorRange(0.0, 10.0); // TODO record typical climb, set to just over typical error accum
            servo = new NeoServo(CANID, posPID, hwVelPID, inverted, SparkFlex.class);
            setParams(CANID, side);            
        }

        private void setParams(int CANID, String side) {
            servo
                .setName("arm-"+side +"-" + CANID)
                .setConversionFactor(conversionFactor) // [cm]
                .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
                .setVelocityHW_PID(maxVel, maxAccel)
                .setTolerance(posTol, velTol)
                .setMaxVelocity(maxVel);
            servo.setPosition(PowerUpPosition);
        }
        
        public void initSendable(SendableBuilder builder) {           
            builder.addDoubleProperty("vel_cmd",  null, this::setVelocity ); //getter must be null
            builder.addDoubleProperty("velocity",  this::getVelocity, null );
            builder.addDoubleProperty("vel_max", servo::getMaxVel, servo::setMaxVelocity);
            posPID.initSendable(builder);
        }

        //Arm API - mostly wrappers around servo
        public void setSetpoint(double pos) {            
            servo.setSetpoint(pos);  //goes into position mode
        }
        
        public double getSetpoint() {
            return servo.getSetpoint();
        }

        public double getPosition() {
            return servo.getPosition();
        }
                
        public void setPosition(double pos){
            servo.setPosition(pos);  //doesn't move, just tells servo here you are.
        }

        public double getVelocityCmd() {
            return servo.getVelocityCmd();
        }

        public void setVelocity(double vel) {
            servo.setVelocityCmd(vel);
        }

        public double getVelocity() {
            return servo.getVelocity();
        }

        public boolean atSetpoint() {
            return servo.atSetpoint();
        }        
    }

    public Climber(boolean oneArm) {
        setName("climber");
        // Set up in this format to use both arms as needed.
        if (oneArm) {
            l_arm = new Arm(CAN.l_arm,"L", false, "Left Arm");
            r_arm = null;
        } else {
            l_arm = new Arm(CAN.l_arm,"L", false, "Left Arm");
            r_arm = new Arm(CAN.r_arm,"R", false, "Right Arm");
        }
        getWatcher();        
    }

    public Command setVelocityCmd(double vel, Arm arm) {
        if (arm == null) {
            return Commands.print("Arm DNE, Could not set velocity");
        }
        return runOnce(() -> {
        arm.setVelocity(vel);  //switches Neo to vel mode
        });
    }

    public Command setVelocityCmd(double vel) {
        return runOnce(() -> {
            l_arm.setVelocity(vel);  //switches Neo to vel mode
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (r_arm != null) {
            r_arm.servo.periodic();
        }
        l_arm.servo.periodic();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        if (r_arm != null) {
             r_arm.initSendable(builder);
        }
        l_arm.initSendable(builder);        
    }
        
    // Climber API - most of these should use both l/r arms together I think
    // May not be necessary, arms will be running separately as they climb up the side of the building

    public Command setSetpointCmd(double pos, Arm arm) {
        if (arm == null) {
            return Commands.print("Arm DNE, could not command position");
        }
        return runOnce(() -> {
        arm.setSetpoint(pos);
        });
         
    }

    public Command armsSetpointCmd(double pos) {
        return runOnce(() -> {  // simple instant cmd, sequenct not needed
            // if we need to move arms at same time, set both arms to same position
            if (r_arm != null) {
                r_arm.setSetpoint(pos);
            } 
            l_arm.setSetpoint(pos);
        });         
    }

    // set both arms to a given position, doesn't move, just initializes to position.
    // typical use at power up or after pitt calibration.
    public Command armsCalibrateCmd(double position) {
        return runOnce(() -> {
            //no sequence needed, these can run in single cmd.
            if (r_arm != null) {
                r_arm.setPosition(position);
            }
            l_arm.setPosition(position);
            
        });
    }

    // Simple check, only used in the arms to point command. 
    public boolean armsAtPos() {
        if (r_arm == null) {
            return l_arm.atSetpoint();
        }
        return l_arm.atSetpoint() && r_arm.atSetpoint();
        
    }


    public Command armsToPoint(double pos) {
        return Commands.sequence(
            armsSetpointCmd(pos),
            Commands.waitUntil(this::armsAtPos),
            Commands.print("Arms are at position")
        ).withName("Arms - " + pos);
    }

    //specify arm of choice if desired. Could cause issues if one of our arms is just flying around, but should still function OK
    public Command armsToPoint(double pos, Arm arm) {
        if (arm == null) {
            return Commands.print("Arm Does not exist, cannot send to point");
        }
        return Commands.sequence(
            setSetpointCmd(pos, arm),
            Commands.waitUntil(arm::atSetpoint),
            Commands.print("Arms are at position")
        ).withName(arm.name + " - " + pos);
    }

    public boolean atSetpoint(){
        if (r_arm == null) {
            return l_arm.atSetpoint();
        }
        return l_arm.atSetpoint() && r_arm.atSetpoint(); 
    }

    public double climbposition() {
        return ExtendPosition;
    }

    public void setPosition(double pos) {
        if (r_arm != null) {
            r_arm.setPosition(pos);
        }
        if (l_arm != null) {
            l_arm.setPosition(pos);
        }
    }

    public Command getWatcher(){
        return this.new ClimberWatcher();
    }

    public void setDemoBindings(CommandXboxController xbox) {
        /**
         * These are some basic test bindings for the climber, including a reset 0 position for when we do position testing.          
        */
        //velocity cmds while held it should spin, to test or align in pitt
        // Got about 70amps at 12cm/s, could hit 14 without issues
        xbox.povLeft().whileTrue(this.setVelocityCmd(-14.0, l_arm)).onFalse(this.setVelocityCmd(0.0, l_arm));
        xbox.povRight().whileTrue(this.setVelocityCmd(14.0, l_arm)).onFalse(this.setVelocityCmd(0.0, l_arm));
        xbox.povUp().whileTrue(this.setVelocityCmd(-14.0, r_arm)).onFalse(this.setVelocityCmd(0.0, r_arm));
        xbox.povDown().whileTrue(this.setVelocityCmd(14.0, r_arm)).onFalse(this.setVelocityCmd(0.0, r_arm));

        // Move arms to 0 point
        xbox.x().onTrue(armsSetpointCmd(0.0)); 
        xbox.a().onTrue(armsToPoint(climbingPos, l_arm)); //using a variable here so we can continue testing

        // tell the arms "here is zero"
        xbox.y().onTrue(armsCalibrateCmd(0.0));
    }
    
    class ClimberWatcher extends WatcherCmd {
        ClimberWatcher() {
            
            addEntry("AtSetpoint", Climber.this::atSetpoint);
            addEntry("L_position", Climber.this.l_arm::getPosition, 1);
            addEntry("Left Arm Motor Current", Climber.this.l_arm.servo.getController()::getOutputCurrent);
            addEntry("L_Velocity", Climber.this.l_arm::getVelocity);
            addEntry("Left Accum Error", Climber.this.l_arm.servo.getController().getClosedLoopController()::getIAccum);
            l_arm.servo.getWatcher();
            if (r_arm != null) {
                // put all Right Arm watchers within
                addEntry("R_position", Climber.this.r_arm::getPosition, 1);
                addEntry("Right arm Motor Current", Climber.this.r_arm.servo.getController()::getOutputCurrent);
                addEntry("R_Velocity", Climber.this.r_arm::getVelocity);
                addEntry("Right Accum Error", Climber.this.r_arm.servo.getController().getClosedLoopController()::getIAccum);
                r_arm.servo.getWatcher();
            }
        }
    }


}