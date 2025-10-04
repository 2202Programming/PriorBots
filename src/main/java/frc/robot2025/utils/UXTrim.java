package frc.robot2025.utils;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class UXTrim {
    //Table to keep all the trims
    final static String tableName = "/TrimTable";
    static NetworkTable TrimTable = NetworkTableInstance.getDefault().getTable(tableName);

    //All the trimable values we are tracking
    static ArrayList<UXTrim> trims = new ArrayList<UXTrim>();

    //instance vars
    final DoubleSubscriber trimSub;
    double trim;
    Supplier<Boolean> changeCallback;

    public UXTrim(String name){
        this(name, 0.0);
    }

    public UXTrim(String name, double default_trim) {     
        DoubleTopic d_topic;
        changeCallback = null;
                
        //create the topic, and subscribe
        d_topic = TrimTable.getDoubleTopic(name);
        trimSub = d_topic.subscribe(default_trim);
        // get the persisted value, if there is one
        this.trim = trimSub.get();
        // publish & persist
        d_topic.publish().set(this.trim);        
        d_topic.setPersistent(true);
    }

    public double getValue() {
        trim = trimSub.get();
        return trim;
    }

    public double getValue(double value){
        trim = trimSub.get();
        return value + trim;
    }

    public UXTrim addChangeCallback(Supplier<Boolean> callback) {
        changeCallback = callback;
        trims.add(this);
        return this;
    }

    //Hook this into the robot loop somewhere 
    static public void periodic() {
        //monitor subs for new trim values if they have a callback
        for (UXTrim uxTrim : trims) {
            double newval = uxTrim.trimSub.get();
            if (newval != uxTrim.trim) {
                uxTrim.trim = newval;
                if (uxTrim.changeCallback != null) 
                    uxTrim.changeCallback.get(); // don't care about returned bool
            }
        }
    }

}
