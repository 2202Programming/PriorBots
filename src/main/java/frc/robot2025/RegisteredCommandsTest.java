package frc.robot2025;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot2025.commands.DriveToPickupTag;
import frc.robot2025.commands.DriveToReefTag;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;

public class RegisteredCommandsTest {
    
    private static Command place(Levels level){
        String name = (level == Levels.LTwo) ? "L2" : "L3";
        return new PrintCommand("place_"+name+"_"+level.toString());
    }
    private static Command place4(Levels level){
        return new PrintCommand("place4" +level.toString());
    }

    public static void RegisterCommands() {
        
        NamedCommands.registerCommand("Pickup",  new PrintCommand("Pickup"));
        NamedCommands.registerCommand("PlaceL4", place4(Levels.LFour));
        NamedCommands.registerCommand("PlaceL3", place(Levels.LThree));
        NamedCommands.registerCommand("PlaceL2", place(Levels.LTwo));
        NamedCommands.registerCommand("PlaceL1", place(Levels.LOne));
        NamedCommands.registerCommand("PickupAdjustment", new PrintCommand("PickupAdjustment"));
       
        NamedCommands.registerCommand("WaitForPickup", new PrintCommand("waiting for pickup").andThen(new WaitCommand(3.0)));  
        NamedCommands.registerCommand("Release", new PrintCommand("Releasing coral"));
        NamedCommands.registerCommand("DriveToReefTagRight", new DriveToReefTag("r")
                        .withDistanceScheduleCmd(new PrintCommand("sched place .8m away"), 0.8));
        NamedCommands.registerCommand("DriveToReefTagLeft", new DriveToReefTag("l"));
        NamedCommands.registerCommand("DriveToPickupTagLeft",new DriveToPickupTag("left"));
        NamedCommands.registerCommand("DriveToPickupTagRight",new DriveToPickupTag("right"));
    }
}

