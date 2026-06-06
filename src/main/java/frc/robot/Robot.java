// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.PathfindingCommand;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private final String dmllName = Constants.LimelightConstants.ll_Name;
    
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }
    
    @Override
    public void robotInit() {
    
        //Setup elastic 
        SmartDashboard.putBoolean("Is Hub active?", false);

        PathfindingCommand.warmupCommand().schedule();
                
    }

    @Override
    public void robotPeriodic() {

        m_timeAndJoystickReplay.update();

        //Setup which pipeline to use
        LimelightHelpers.setPipelineIndex(dmllName, m_robotContainer.getPipeline());

        // Retrieves the voltage currently entering the roboRIO
        double batteryVoltage = RobotController.getBatteryVoltage();
        
        //Where the logs will go
        SignalLogger.setPath("/home/lvuser/logs/");
    
        // Sends the value to SmartDashboard under the key "Battery Voltage"
        SmartDashboard.putNumber("Battery Voltage", batteryVoltage);
        CommandScheduler.getInstance().run(); 
        SmartDashboard.putBoolean("Is Hub active?", isHubActive());
        
    }

    @Override
    public void disabledInit() {

        //Adds throttling(slowdown) of the limelight to prevent overheating
        NetworkTableInstance.getDefault().getTable(dmllName).getEntry("throttle_set").setNumber(100);
    }

    @Override
    public void disabledPeriodic() {
        //Sets up the orginal orientation of the robot
        LimelightHelpers.SetIMUMode(dmllName, 1); // Seed internal IMU (1)
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        //Removes throttling(slowdown) of the limelight
        NetworkTableInstance.getDefault().getTable(dmllName).getEntry("throttle_set").setNumber(0);
    }

    @Override
    public void autonomousPeriodic() {
        
        //Set megatag 2 IMU mode (not used anymore)
        LimelightHelpers.SetIMUMode(dmllName, 0); //External only
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {

        //Removes throttling(slowdown) of the limelight
        NetworkTableInstance.getDefault().getTable(dmllName).getEntry("throttle_set").setNumber(0);

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {

        //Set megatag 2 IMU mode (not used anymore)
        LimelightHelpers.SetIMUMode(dmllName, 4); //Internal IMU + External IMU
        LimelightHelpers.SetIMUAssistAlpha(dmllName, 0.001); //// Set the complementary filter alpha
    }

    @Override
    public void teleopExit() {}
    
    //Other stuff that isn't the robot


    //Check if hub is active
    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
            // If we have invalid game data, assume hub is active.
            return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }
}
