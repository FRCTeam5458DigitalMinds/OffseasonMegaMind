package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase{
    private PowerDistribution sparkPDH = new PowerDistribution(Constants.LEDConstants.PDH, ModuleType.kRev);

    //Turns on LED 
    public Command LEDon(){
     return run(() -> {turnOn();});
    }
  
    //Turns off LED 
    public Command LEDoff(){
     return run(() -> {turnOff();});
    }

    //Function version of turn off LED
    public void turnOff(){
        sparkPDH.setSwitchableChannel(false);

    }

    //Function version of turn on LED
    public void turnOn(){
        sparkPDH.setSwitchableChannel(true);
    }
    
    //Unused- Giovan
    //Alternates on and off
    public Command blink(){
        return Commands.sequence( 
            LEDon()
            .andThen(Commands.waitSeconds(0.25))
            .andThen(LEDoff())
            .andThen(Commands.waitSeconds(0.25))
        );
    }

    //Used - Giovan
    public void showTagStatus(boolean tagVisible) {

        if (tagVisible) {
            turnOn();     // LED red when tag detected
        } 
        else {
            turnOff();    // LED off when no tag
        }
    }
}
