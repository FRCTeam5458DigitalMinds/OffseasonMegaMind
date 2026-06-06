// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    TalonFX climberMotor;

    //Min (0), max (1), climb down (2)
    private final double[] setpoints = {0,139.50048828125,68};

    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public Climber() {
        climberMotor = new TalonFX(Constants.ClimbConstants.climbMotor);

        TalonFXConfiguration climbConfigs = new TalonFXConfiguration();
        //setups the PID value for the intake
        climbConfigs.Slot0.kP = Constants.ClimbConstants.climb_P;
        climbConfigs.Slot0.kD = Constants.ClimbConstants.climb_D; 

        climbConfigs.CurrentLimits.withStatorCurrentLimit(40);
        climbConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);


        climberMotor.getConfigurator().apply(climbConfigs);

        //Set idle mode
        climberMotor.setNeutralMode(NeutralModeValue.Brake);

        //Reset encoder
        climberMotor.setPosition(0);

    }

    public void setClimber(double OutputPercent){
        OutputPercent /= 100.0;
        climberMotor.set(OutputPercent);
    }

    public Command runMotors(double OutputPercent){
      return run(
          () -> {
            setClimber(OutputPercent);
          }
      );
    }

    public Command stopMotors(){
      return run(
          () -> {
            setClimber(0);
          }
      );
    }

     //Go to certain position based on setpoint index
    public Command toSetpoint(int setpointIndex)
    {
        return run(
            () -> {climberMotor.setControl(m_request.withPosition(setpoints[setpointIndex]).withSlot(0));}
        );
    }

    //Go to position not based on a setpoint
    public void customPosition(double setPoint)
    {
        climberMotor.setControl(m_request.withPosition(setPoint).withSlot(0));
    }

    //more testing
    public double getPosition()
    {
        double climbEncoder = climberMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Climb Position", climbEncoder);
        return climbEncoder;
    }

    //Potenial climb command
    /*public Command climb(){
        return Commands.sequence(
            toSetpoint(2).until(::readytoClimb)
            .andThen(Commands.waitSeconds(0.25))
            .andThen(toSetpoint(3))
        ); 
    }*/
    
    //Checks if climber is high enough
    public boolean readytoRest(){
        return getPosition() > 139.3;
    }
    
}
