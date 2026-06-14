// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  private TalonFX intakeMotor;

  //max extenstion, min extension, other number...
  private final double[] setpoints = {14.00833984375,-3.219970703125};

  //Set up position PID
  private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  public Intake() {

    //Set up motors
        intakeMotor = new TalonFX(Constants.IntakeConstants.intakeMotor);
        
        TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        //setups the PID value for the intake
        intakeConfigs.Slot0.kP = Constants.IntakeConstants.intake_P;
        intakeConfigs.Slot0.kD = Constants.IntakeConstants.intake_D; 

        intakeConfigs.Slot1.kP = Constants.IntakeConstants.intake_P-0.5; //Never used

        //Setups current limits
        intakeConfigs.CurrentLimits.withStatorCurrentLimit(40);
        intakeConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);

        intakeMotor.getConfigurator().apply(intakeConfigs);

        //Resets encoder to this value
        intakeMotor.setPosition(-3.219970703125);

  }

    //Used in comp - Giovan
    //Sets speed of Intake
    public void setIntake(double OutputPercent)
    {
      OutputPercent /= 100.;

      //Negative means extends
      intakeMotor.set(-OutputPercent);
    }

    //Used in Comp. - Giovan
    //Go to certain position based on setpoint index
    public void toSetpoint(int setpointIndex)
    {
        intakeMotor.setControl(m_request.withPosition(setpoints[setpointIndex]).withSlot(0));
    }

    //Unused - Giovan
    //Go to setpoint but slower 
    public void slowerToSetpoint(int setpointIndex)
    {
        intakeMotor.setControl(m_request.withPosition(setpoints[setpointIndex]).withSlot(1));
    }

    //Used in comp - Giovan
    //Go to position not based on a setpoint
    public void customPosition(double setPoint)
    {
        intakeMotor.setControl(m_request.withPosition(setPoint).withSlot(0));
    }

    //Unused - Giovan
    //go to position but slower
    public void slowerCustomPosition(double setPoint){
        intakeMotor.setControl(m_request.withPosition(setPoint).withSlot(1));

    }
  
    //Testing purposes - Giovan
    //Get encoder value of the intake
    public double getPosition()
    {
        double intakeEncoder = intakeMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Intake Position", intakeEncoder);
        return intakeEncoder;
    }

    /* 
    Look up debouncer for motor stalling
    */

    //Moves in based on current position
    public Command retractIntake(){
      return runOnce(
        () -> {customPosition(getPosition()-8);}
      );
    }


    //Slower version of the function above
    public Command SlowretractIntake(){
      return runOnce(
        () -> {slowerCustomPosition(getPosition()-10);}
      );
    }

    //Moves out based on current position
    public Command extendIntake(){
      return runOnce(
        () -> {customPosition(getPosition()+8);}
      );
    }

    //Slower version of the function above
    public Command SlowextendIntake(){
      return runOnce(
        () -> {slowerCustomPosition(getPosition()+10);}
      );
    }
    
    //Slowly brings intake
    public Command slowRetract(){
      return runEnd(
        () -> {setIntake(7);},
        () -> {setIntake(0);}
      ); 
    }

    //Both stopped using
    //Dev code, unused - Giovan
    public boolean atEnd(){
      return getPosition() < setpoints[1]+((setpoints[1]+Math.abs(setpoints[0]))/2);
    }
    public boolean AutoatEnd(){
      return getPosition() < setpoints[1]+((setpoints[1]+Math.abs(setpoints[0]))/4);
    }

    /*public Command resetPosition() {
      return intakeMotor.setPosition(-3.219970703125)
    }*/
}
