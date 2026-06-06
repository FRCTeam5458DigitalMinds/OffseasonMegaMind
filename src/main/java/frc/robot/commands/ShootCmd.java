package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

//Not used - Giovan
public class ShootCmd extends Command {

    Hood HOOD;
    Shooter SHOOTER;
    


    public ShootCmd(/*Add needed subsystems */) 
    {
        //Give value to declared subsystems

        //Could also add subsystem requirements
    
    }

    public void initialize()
    {   
        //run once        
    }

    public void execute()
    {
        
        //will keep running until finished

        //runs "isFinished" function to say how it is done
        isFinished();
    }


    //sets boolean for its done to stop the comand
    public boolean isFinished()
    {
      return true;
    }

    public void end(boolean interupted){
    }
}