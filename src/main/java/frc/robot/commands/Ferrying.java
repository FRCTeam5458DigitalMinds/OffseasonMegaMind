package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

//not used -giovan
public class Ferrying extends Command {

    Hood HOOD;

    public Ferrying(Hood hood) 
    {
        this.HOOD = hood;
        addRequirements(getRequirements());
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

    //public boolean end(something interupted){
    //} 
}