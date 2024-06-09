package frc.lib2202.subsystem.Swerve.Config;

/* WIP - for now placeholder for code and comments pulled out of SDT 
 * 
 *  Will likely used DTMonitorCmd to allow PIDF objects to be edited and this will be deleted.
*/
public class ChassisPIDFTuning {
    // below is commented out code from SDT.java
    /*
     * double drive_kP = DriveTrain.drivePIDF.getP();
     * double drive_kI = DriveTrain.drivePIDF.getI();
     * double drive_kD = DriveTrain.drivePIDF.getD();
     * double drive_kFF = DriveTrain.drivePIDF.getF();
     * 
     * double angle_kP = DriveTrain.anglePIDF.getP();
     * double angle_kI = DriveTrain.anglePIDF.getI();
     * double angle_kD = DriveTrain.anglePIDF.getD();
     * double angle_kFF = DriveTrain.anglePIDF.getF();
     */

    // display PID coefficients on SmartDashboard if tuning drivetrain
    /*
     * SmartDashboard.putNumber("Drive P", drive_kP);
     * SmartDashboard.putNumber("Drive I", drive_kI);
     * SmartDashboard.putNumber("Drive D", drive_kD);
     * SmartDashboard.putNumber("Drive Feed Forward", drive_kFF);
     * 
     * SmartDashboard.putNumber("Angle P", angle_kP);
     * SmartDashboard.putNumber("Angle I", angle_kI);
     * SmartDashboard.putNumber("Angle D", angle_kD);
     * SmartDashboard.putNumber("Angle Feed Forward", angle_kFF);
     */

    // Move to a TEST/Tuning command - DPL 2/21/22
    // private void pidTuning() { //if drivetrain tuning

    // // read PID coefficients from SmartDashboard if tuning drivetrain
    // double drive_p = SmartDashboard.getNumber("Drive P Gain",
    // DriveTrain.drivePIDF.getP());
    // double drive_i = SmartDashboard.getNumber("Drive I Gain",
    // DriveTrain.drivePIDF.getI());
    // double drive_d = SmartDashboard.getNumber("Drive D Gain",
    // DriveTrain.drivePIDF.getD());
    // double drive_ff = SmartDashboard.getNumber("Drive Feed Forward",
    // DriveTrain.drivePIDF.getF());
    // double angle_p = SmartDashboard.getNumber("Angle P Gain",
    // DriveTrain.anglePIDF.getP());
    // double angle_i = SmartDashboard.getNumber("Angle I Gain",
    // DriveTrain.anglePIDF.getI());
    // double angle_d = SmartDashboard.getNumber("Angle D Gain",
    // DriveTrain.anglePIDF.getD());
    // double angle_ff = SmartDashboard.getNumber("Angle Feed Forward",
    // DriveTrain.anglePIDF.getF());

    // // if anything changes in drive PID, update all the modules with a new drive
    // PID
    // if ((drive_p != drive_kP) || (drive_i != drive_kI) || (drive_d != drive_kD)
    // || (drive_ff != drive_kFF)) {
    // drive_kP = drive_p;
    // drive_kI = drive_i;
    // drive_kD = drive_d;
    // drive_kFF = drive_ff;
    // for (SwerveModuleMK3 i : modules) {
    // i.setDrivePID(new PIDFController(drive_kP, drive_kI, drive_kD, drive_kFF));
    // }
    // }

    // // if anything changes in angle PID, update all the modules with a new angle
    // PID
    // if ((angle_p != angle_kP) || (angle_i != angle_kI) || (angle_d != angle_kD)
    // || (angle_ff != angle_kFF)) {
    // angle_kP = angle_p;
    // angle_kI = angle_i;
    // angle_kD = angle_d;
    // angle_kFF = angle_ff;
    // for (SwerveModuleMK3 i : modules) {
    // i.setAnglePID(new PIDFController(angle_kP, angle_kI, angle_kD, angle_kFF));
    // }
    // }
    // }

}
