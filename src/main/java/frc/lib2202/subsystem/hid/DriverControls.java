package frc.lib2202.subsystem.hid;

@Deprecated
public class DriverControls {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public enum DriverMode {
    Arcade(0), Tank(1), XYRot(2);

    public final int value;

    DriverMode(int value) {
      this.value = value;
    }
  }
}