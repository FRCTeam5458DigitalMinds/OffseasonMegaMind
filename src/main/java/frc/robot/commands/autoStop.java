package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.*;

//Not used - giovan
public class autoStop extends Command {

    Feeder FEEDER;
    Indexer INDEXER;
    Shooter SHOOTER;
    Hood HOOD;
    Roller ROLLER;
    Intake INTAKE;

    public autoStop(Feeder feeder, Indexer indexer, Shooter shooter, Hood hood, Roller roller, Intake intake) 
    {
        this.FEEDER = feeder;
        this.INDEXER = indexer;
        this.SHOOTER = shooter;
        this.HOOD = hood;
        this.ROLLER = roller;
        this.INTAKE = intake;

        addRequirements(FEEDER);
        addRequirements(INDEXER);
        addRequirements(SHOOTER);
        addRequirements(HOOD);
        addRequirements(ROLLER);
        addRequirements(INTAKE);    
    }

    public void initialize()
    {   
        Commands.parallel(
            INTAKE.runOnce(() -> {INTAKE.toSetpoint(1);}),
            INDEXER.setSpeed(0),
            SHOOTER.stopMotors(),
            HOOD.toSetpoint(0),
            ROLLER.setSpeed(0),
            FEEDER.setSpeed(0) 
        );
        isFinished();
    }

    //sets boolean for its done to stop the comand
    public boolean isFinished()
    {
      return true;
    }
}