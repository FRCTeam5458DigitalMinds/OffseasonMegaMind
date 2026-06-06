package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

//v1
//prototype autoalign - giovan
public class AutoalignRotate extends Command {

    Limelight LIMELIGHT;
    CommandSwerveDrivetrain DRIVETRAIN;
    SwerveRequest.RobotCentric robotDrive;


    Double maxAnglSpeed; //Max Angular Speed

    public AutoalignRotate(Limelight limelight, CommandSwerveDrivetrain drivetrain, Double maxAngularSpeed) 
    {
        this.LIMELIGHT = limelight;
        this.DRIVETRAIN = drivetrain;
        robotDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        maxAnglSpeed = maxAngularSpeed;

        addRequirements(limelight);
        addRequirements(drivetrain);
    }

    //puts the Target ID on the Dashboard
    public void initialize()
    {   
        DRIVETRAIN.setControl(
            robotDrive.withRotationalRate(0)
                      .withVelocityX(0)
                      .withVelocityY(0)
        );
        SmartDashboard.putNumber("ID", LIMELIGHT.getTargetID());
    }

    //runs 
    public void execute()
    {
        double currentTXNC = 0.0;
        //gets tx from target id, autoaligns, keeps looping until tx centered

            //gets raw fiducial (again)
        
            RawFiducial[] crtFiducials = LIMELIGHT.getFiducialData();

      //moves raw fiducial to current target ids, takes only tx of the current target id    
            for (RawFiducial fiducial : crtFiducials) {

                if (fiducial.id == LIMELIGHT.priorityTag()){
                    currentTXNC = fiducial.txnc; // X offset (no crosshair)
                }
            }

            if (LIMELIGHT.priorityTag() != 0){

                    //Runs function to get turn speed
                    double turnSpeed = LIMELIGHT.limelight_aim_proportional(maxAnglSpeed,currentTXNC);
                    SmartDashboard.putNumber("TXNC", currentTXNC);

                    //Uses turn speed to run robot
                    DRIVETRAIN.setControl(robotDrive.withRotationalRate(turnSpeed));

            }
            else {
                isFinished();
            }

        //runs "isFinished" function to say how it is done
    }

    public void end(boolean interupted){
        DRIVETRAIN.setControl(robotDrive.withRotationalRate(0));
    }
}