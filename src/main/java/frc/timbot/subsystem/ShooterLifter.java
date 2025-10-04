/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.timbot.subsystem;

import static frc.lib2202.Constants.DEGperRAD;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.util.PIDFController;
import frc.timbot.Constants.CAN;
import frc.timbot.Constants.PCM;
import frc.timbot.Constants.AnalogIn;

public class ShooterLifter extends SubsystemBase {
  /**
   * SS to that raises and lowers the shooter frisbe platform.
   * 
   * actuator  - SparkMax to control a linear actuator to lift platform
   * pos_volts - voltage from actuator read via analogInput
   * trigger   - double solenoid
   * 
   */

   //physical consts
   final double K_h = 32.0; // [cm] tbd lengh of platform piviot (hypotenuse)
   final double K_h_offset = 0.0; //offset at zero deg
   final double K_VperCM = 1.2 * 2.54; // basiclly Vin[Volts * 6.0 inches)/5.0Volts] * 2.54 cm/in

  // pid gains for motor controller
  double kp = 20.0;  // [%pwr/cm]
  double ki = 0.0;
  double kd = 0.0;
  final PIDFController controller = new PIDFController(kp, ki, kd, 0.0);

  //devices used
  final SparkMax actuator;
  final AnalogInput pos_volts; 
  final DoubleSolenoid trigger;

  // Lifter state vars
  double height_meas = 0.0;  //updated in periodic

  public ShooterLifter() {
    actuator = new SparkMax(CAN.ACTUATOR, SparkMax.MotorType.kBrushed);
    trigger = new DoubleSolenoid(PneumaticsModuleType.REVPH, PCM.TRIGGER_BACK, PCM.TRIGGER_FORWARD);
    pos_volts = new AnalogInput(AnalogIn.LifterFeedback);
    // configure A/D behavior on Rio
    pos_volts.setAverageBits(4);
  }

  @Override
  public void periodic() {
    height_meas = getHeight();  //measures feedback
    // run our pid around the h_cmd,  speed is on [-1, 1.0], controlled by pidf gains, kp
    double actuator_speed = controller.calculate(height_meas);
    actuator_speed = MathUtil.clamp(actuator_speed, -1.0, 1.0);  // min/max % power
    actuator.set(actuator_speed);  //using simple %power mode
  }

  // Platform API
  public double getHeight() { // in cm
    // System.out.println(volts.getVoltage());
    return K_VperCM * pos_volts.getVoltage();
  }

  // height in [cm]
  public void setHeight(double height_cm) {
    controller.setSetpoint(height_cm); 
  }

  // alternative way to set postion
  public void setAngle(double angle_deg) {
    // convert angle to cm
    double height_cmd = K_h*Math.sin(angle_deg) + K_h_offset;
    setHeight(height_cmd);   
  }

  public double getAngle() {
    double h = getHeight() - K_h_offset;
    double angle = Math.asin(h/K_h)*DEGperRAD;
    return angle;
  }
  
  //Trigger API
  public void trigger_fire(){
    trigger.set(DoubleSolenoid.Value.kForward);
  }
  public void trigger_reset(){
    trigger.set(DoubleSolenoid.Value.kReverse);
  }

}
