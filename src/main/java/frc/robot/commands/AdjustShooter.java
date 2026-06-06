package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.*;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


//Not used - Giovan
public class AdjustShooter extends Command {

    Hood HOOD;
    Shooter SHOOTER;
    Limelight LIMELIGHT;

    public AdjustShooter(Hood hood, Shooter shooter, Limelight limelight) 
    {
        this.HOOD = hood;
        this.SHOOTER = shooter;
        this.LIMELIGHT = limelight;

        addRequirements(hood);
        addRequirements(shooter);
        addRequirements(limelight);
        
    }

    public void initialize()
    {          
        HOOD.toTreeSetpoint(LIMELIGHT.getDistToNearestTag());
        //Add shooter part later

    }

    public void execute()
    {
        //SmartDashboard.putNumber("Predicted Hood angle",hoodAngle.get(LIMELIGHT.getDistToNearestTag()));
        //SmartDashboard.putNumber("Predicted Shooter RPS",shooterRPS.get(LIMELIGHT.getDistToNearestTag()));
   
        /*//will keep running until finished
        HOOD.customPosition(hoodAngle.get(LIMELIGHT.getDistToNearestTag()));
        SHOOTER.PIDrunMotors(shooterRPS.get(LIMELIGHT.getDistToNearestTag()));*/
    }
    public void end(boolean interupted){
        SHOOTER.stopMotors();
    }

}