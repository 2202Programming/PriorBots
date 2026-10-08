package frc.robot2026.subsystems.Shooter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.WatcherCmd;
import frc.robot2026.Constants.DigitalIO;

public class Fuelgauge extends SubsystemBase {
    public static Trigger NoFuelTrigger;
    
    final double FUEL_DELAY = 0.300; // 300ms

    final Indexer idx_left, idx_right;
    final DigitalInput gauge;
    final Timer fuel_timer;

    boolean empty = true;

    public Fuelgauge() {
        gauge = new DigitalInput(DigitalIO.FuelGauge);
        // indexers_have_fuel must exist or the next two lines with throw NPE
        this.idx_left = RobotContainer.getSubsystem("indexer_left");
        this.idx_right = RobotContainer.getSubsystem("indexer_right");
        fuel_timer = new Timer();
        fuel_timer.start();

        // create a static trigger object tied to this instance
        NoFuelTrigger = new Trigger(this::isEmpty);

        this.new FuelGaugeWatcher();   //TODO comment out once debugged
    }

    public boolean isEmpty() {
        return empty;
    }

    boolean hasFuel() {
        //returns true if fuel is blocking gauge
        return !gauge.get(); 
    }

    public void periodic() {
        // idx.isLoaded() latches having a fuel, hasFuel is only the light gate so 
        // it misses when a fuel is backed off of gate during Load() command.
        boolean indexers_have_fuel = idx_left.isLoaded() || idx_right.isLoaded();

        if (hasFuel() || indexers_have_fuel) {
            // restart the timer because we've seen fuel
            fuel_timer.restart();
        }

        empty = fuel_timer.hasElapsed(FUEL_DELAY) && !indexers_have_fuel;
    }

    public class  FuelGaugeWatcher extends WatcherCmd{
        FuelGaugeWatcher() {
            addEntry("_Empty_", Fuelgauge.this::isEmpty);
            addEntry("HasFuel", Fuelgauge.this::hasFuel);
            addEntry("Loaded_Left", Fuelgauge.this.idx_left::isLoaded);
            addEntry("Loaded_Right", Fuelgauge.this.idx_right::isLoaded);
            addEntry("Timer", Fuelgauge.this.fuel_timer::get, 2);
        }
        
    }

}
