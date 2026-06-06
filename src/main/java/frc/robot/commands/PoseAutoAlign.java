package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.*;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

//Used! - Giovan
public class PoseAutoAlign extends Command {

    CommandSwerveDrivetrain DRIVETRAIN;
    SwerveRequest.FieldCentric robotDrive;
    public double targetRotation;

    public double AngleError;
    
    //where shooter is on robot
    private static final Translation2d shooterOffset = new Translation2d(Inch.of(9).in(Meters), 0.184);

    public PoseAutoAlign(CommandSwerveDrivetrain drivetrain) {
        this.DRIVETRAIN = drivetrain;
        robotDrive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        addRequirements(drivetrain);
    }

    public void execute() {

        //Get drivetrain pose
        Pose2d pose = DRIVETRAIN.getPose();

        //Setup shooter pose
        Translation2d shooterPose = pose.getTranslation().plus(shooterOffset.rotateBy(pose.getRotation()));

        //Get difference between Hub and shooter pose
        Translation2d toHub = DRIVETRAIN.setHub().minus(shooterPose);

        //Get the angle from the x-axis (vertical line)
        targetRotation = toHub.getAngle().getRadians();

        //Difference between target rotation and pose rotation
        AngleError = targetRotation - pose.getRotation().getRadians();

        //Wrap number between negative and positve pi
        AngleError = MathUtil.angleModulus(AngleError);

        AngleError *= 3.5;

        // Clamp max speed
        AngleError = MathUtil.clamp(AngleError, -6, 6);

        // Add minimum output (helps small-angle response)
        if (Math.abs(AngleError) < 0.15 && Math.abs(AngleError) > Math.toRadians(0.5)) {
            AngleError = Math.copySign(0.15, AngleError);
        }
        //Runs drivetrain with angle error
        DRIVETRAIN.setControl(
            robotDrive.withVelocityX(0)
                      .withVelocityY(0)
                      .withRotationalRate(AngleError)
        ); 
        
    }   

    //Once turned near hub, stop
    @Override
    public boolean isFinished() {
        return (Math.abs(AngleError) < Math.toRadians(0.5));
    }

    //If interrupted, stop drivetrain
    @Override
    public void end(boolean interrupted){
        DRIVETRAIN.setControl(
            new SwerveRequest.Idle() 
        );  
    }
}