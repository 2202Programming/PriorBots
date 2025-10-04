package frc.timbot.commands;

public class LifterToggle extends LifterMove {

    final double h_min = 0.0; //[cm]
    final double h_max = 6*2.54; //[cm]

    public LifterToggle()  {
        super(0.0);  // we override initialize() so this arg doesn't matter
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_lifter.getHeight() >  0.8 * h_max /*cm */) {
            m_lifter.setHeight(h_min);
        } else {
            m_lifter.setHeight(h_max);
        }
    }

    //command will finish when at the set point, see MoveLifter
}
