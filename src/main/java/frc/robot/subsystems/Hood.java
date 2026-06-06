// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  TalonFX hoodMotor;
  //min, max, max for hub shooting
  //66 degrees, N/A, 11 degrees

  private final double[] setpoints = {-0.05712890625, 10.63232421875, 4.011962890625};
  private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  private final InterpolatingDoubleTreeMap hoodAngle;
  private final InterpolatingDoubleTreeMap NewhoodAngle;



  public Hood() {
        hoodMotor = new TalonFX(Constants.HoodConstants.hoodMotor);

        TalonFXConfiguration hoodConfigs = new TalonFXConfiguration();

        //setups the PID value for the intake
        hoodConfigs.Slot0.kP = Constants.HoodConstants.hood_P;
        hoodConfigs.Slot0.kD = Constants.HoodConstants.hood_D; 

        hoodConfigs.CurrentLimits.withStatorCurrentLimit(40);
        hoodConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);

        hoodMotor.getConfigurator().apply(hoodConfigs);

        //Resets Hood encoder
        hoodMotor.setPosition(-0.05712890625);


        /* 
          Context to Interpolating Double Tree Map:
          
            *Creates a plot of data points
            *Uses data points to estimate value based off key
        */

        //Sets up data table
        hoodAngle = new InterpolatingDoubleTreeMap();

        //Key: distance
        //Value: hood angle
       
        //Old data points as of 3/7/2026
        hoodAngle.put(0.8732327907316006,-0.06005859375);
        hoodAngle.put(1.1529219342235464,2.23974609375);
        hoodAngle.put(1.8059915426872029,5.30224609375);
        hoodAngle.put(2.5236079021737163,6.87060546875);
        hoodAngle.put(4.226945331446267,3.01318359375);
        
        
        //New table for later
        NewhoodAngle = new InterpolatingDoubleTreeMap();
        
    }

    //Set speed of Hood motor
    public void setHood(double OutputPercent){
        OutputPercent /= 100.0;
        hoodMotor.set(OutputPercent);
    }

    //Go to certain position based on setpoint index
    public Command toSetpoint(int setpointIndex)
    {
        return run(
            () -> {hoodMotor.setControl(m_request.withPosition(setpoints[setpointIndex]).withSlot(0));}
        );
    }

    //Unused - Giovan
    //Go to a hood position based on distance from tag
    public Command toTreeSetpoint(double distance){
        return runOnce(
            () -> {goToPostion(distance);}
        );
    }

    //Function version of Command above
    //Plus put predicted Hood angle in Dashboard
    public void goToPostion(double distance){
        hoodMotor.setControl(m_request.withPosition(hoodAngle.get(distance)).withSlot(0));
        SmartDashboard.putNumber("Predicted Hood angle", hoodAngle.get(distance));
    }

    //Testing purposes only - Giovan
    //Go to position not based on a setpoint
    public void customPosition(double setPoint)
    {
        hoodMotor.setControl(m_request.withPosition(setPoint).withSlot(0));
    }
    
    //Testing purposes only - Giovan
    //Get encoder value of the Hood
    public double getPosition()
    {
        double encoder = hoodMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Hood encoder", encoder);
        return encoder;
    }

}
