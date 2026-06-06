package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.*;

//dev code, not used - giovan 
public class unclimb extends Command {

    CommandSwerveDrivetrain DRIVETRAIN;
    SwerveRequest.RobotCentric robotDrive;

    Climber CLIMBER;

    public unclimb(CommandSwerveDrivetrain drivetrain, Climber climber) 
    {
        this.DRIVETRAIN = drivetrain;
        robotDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.CLIMBER = climber;

        addRequirements(drivetrain);
        addRequirements(climber);
    }

    public void initialize()
    {   
        Commands.sequence(
            CLIMBER.toSetpoint(1)
            .andThen(
                DRIVETRAIN.runOnce(
                    () -> {DRIVETRAIN.setControl(robotDrive.withVelocityX(-1));}
                )
            )
            .andThen(Commands.waitSeconds(0.1))
            .andThen(DRIVETRAIN.runOnce(
                    () -> {DRIVETRAIN.setControl(robotDrive.withVelocityX(0));}
                )
            )
            .andThen(CLIMBER.toSetpoint(0))
        );
        isFinished();    
    }

    //sets boolean for its done to stop the comand
    public boolean isFinished()
    {
      return true;
    }
 
}