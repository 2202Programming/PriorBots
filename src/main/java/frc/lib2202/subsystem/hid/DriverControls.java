package frc.lib2202.subsystem.hid;

public class DriverControls {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public enum Id {
    Driver(0), Operator(1), SwitchBoard(2), Phantom(3);

    public final int value;

    Id(int value) {
      this.value = value;
    }
  }

  public enum DriverMode {
    Arcade(0), Tank(1), XYRot(2);

    public final int value;

    DriverMode(int value) {
      this.value = value;
    }
  }
}