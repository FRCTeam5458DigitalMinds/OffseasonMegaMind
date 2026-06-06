// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Roller extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  private TalonFX RollerMotor;
  
  public Roller() {

    //Set up motors
        RollerMotor = new TalonFX(Constants.IntakeConstants.rollerMotor);       
        //
        TalonFXConfiguration RollerConfigs = new TalonFXConfiguration();
        RollerConfigs.CurrentLimits.withStatorCurrentLimit(40);
        RollerConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);

        RollerConfigs.TorqueCurrent.withPeakForwardTorqueCurrent(20)
            .withPeakReverseTorqueCurrent(-20);

        RollerMotor.getConfigurator().apply(RollerConfigs);

  }
  public Command setSpeed(double OutputPercent){
    return runOnce(
      () -> {
        setRollers(OutputPercent);
      }
    );
  }

   public void setRollers(double OutputPercent)
    {
      OutputPercent /= 100.;
      RollerMotor.set(OutputPercent);
    }

}
