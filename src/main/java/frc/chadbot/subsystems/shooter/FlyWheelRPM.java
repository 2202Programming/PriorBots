package frc.chadbot.subsystems.shooter;

public class FlyWheelRPM {
    /**
   *  helper class to configure both shooter flywheel speeds
   */
    public double upper;
    public double lower;

    public FlyWheelRPM() { this(0.0, 0.0); }
    public FlyWheelRPM(FlyWheelRPM c) { this(c.lower, c.upper); }
    public FlyWheelRPM(double lower, double upper) {
      this.upper = upper;
      this.lower= lower;
    }

    public double getAverage() { 
      return (0.5*(upper + lower));
    }
    /**
     * Copy(src) - copy data into structure from a source
     * @param src
     */
    public FlyWheelRPM copy(FlyWheelRPM src) {
      this.upper = src.upper;
      this.lower = src.lower;
      return this;
    }

    public FlyWheelRPM set( double upper, double lower) {
      this.upper = upper;
      this.lower= lower;
      return this;
    }

    public FlyWheelRPM minus(FlyWheelRPM a, FlyWheelRPM b) {
      this.upper = a.upper - b.upper;
      this.lower = a.lower - b.lower;
      return this;
    }
    public String toString() {
      return Double.toString(upper) + "/" + Double.toString(lower);
    }
  }
