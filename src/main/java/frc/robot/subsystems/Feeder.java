// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;


import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volt;


public class Feeder extends SubsystemBase {
    
    //Set up motor
    private TalonFX feedMotor;

    //Bang Bang setup
    private BangBangController controller = new BangBangController();

    Double testRPS;

    //sets up velocity PID for slot 0
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    //Set up SysID
    private final SysIdRoutine m_sysIdRoutine;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    //Feedfoward setup
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.FeederConstants.kS,Constants.FeederConstants.kV,Constants.FeederConstants.kA);

    private double lastKP = -1;

    public Feeder() {
        
        feedMotor = new TalonFX(Constants.FeederConstants.feeder);
        TalonFXConfiguration feedConfigs = new TalonFXConfiguration();
        feedConfigs.Slot0.kS = Constants.ShooterConstants.kS;
        feedConfigs.Slot0.kV = Constants.ShooterConstants.kV;
        feedConfigs.Slot0.kA = Constants.ShooterConstants.kA;
        feedConfigs.Slot0.kP = Constants.ShooterConstants.p_Value;


        feedConfigs.CurrentLimits.withStatorCurrentLimit(60);
        feedConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        
        feedMotor.getConfigurator().apply(feedConfigs);
        
        feedMotor.setNeutralMode(NeutralModeValue.Coast); 
        
        //Default RPS
        SmartDashboard.putNumber("Feeder Test RPS", 75.13);
        //45

        SmartDashboard.putNumber("Feeder P_value", Constants.FeederConstants.p_Value);


        //Get kV,kS,kA,kP values
        m_sysIdRoutine = new SysIdRoutine(
              new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
                  Volt.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
          null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("Feed state", state.toString())
        ),
          new SysIdRoutine.Mechanism(
          (volts) -> feedMotor.setControl(m_voltReq.withOutput(volts.in(Volt))),
          null,
          this
        )
      );
 
    }

    //SysID Cmds
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.dynamic(direction);
    }
    

    //Sets speed of feeder
    public Command setSpeed(double OutputPercent){
      return run(
          () -> {
            setFeeder(OutputPercent);
          });
    }

    //PID: runs desired RPS
    public Command PIDtreeRunMotors(){
      return run(
          () -> {
            setTargetRPS(-testRPS);
          }
      );
    }

    //Bang Bang: runs desired RPS
    public Command BBtestMotors(){
      return run(
        () -> {
          BBrps(-testRPS);
        }
      );
    } 

    //Stop motors
    public Command stopMotors(){
      return run(
        () -> {
          setFeeder(0);
        }
      );
    } 


    //Bang Bang
    public void BBrps(double RPS){
        feedMotor.setVoltage(controller.calculate(feedMotor.getVelocity().getValueAsDouble(), RPS)*12 + 0.9*feedforward.calculate(RPS));
    }

    //PID
    public void setTargetRPS(double RPS){
      SmartDashboard.putNumber("RPS", feedMotor.getVelocity().getValueAsDouble());
      feedMotor.setControl(m_request.withVelocity(RPS));
    }


    //Old set speed
     public void setFeeder(double OutputPercent)
    {
      OutputPercent /= 100.;
      feedMotor.set(-OutputPercent);
    }

    //Continuously runs
    @Override
    public void periodic() {

      //Sets test RPS
      testRPS = SmartDashboard.getNumber("Feeder Test RPS", 75.13);

      //Current Feeder RPS
      SmartDashboard.putNumber("Feeder RPS", feedMotor.getVelocity().getValueAsDouble());


      double newKP = SmartDashboard.getNumber("Feeder P_value", Constants.ShooterConstants.p_Value);

      if (newKP != lastKP) {
            TalonFXConfiguration globalConfigs = new TalonFXConfiguration();
            globalConfigs.CurrentLimits.withStatorCurrentLimit(60);
            globalConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
            
            //setups the PID value for the intake
          
            globalConfigs.Slot0.kS = Constants.ShooterConstants.kS;
            globalConfigs.Slot0.kV = Constants.ShooterConstants.kV;
            globalConfigs.Slot0.kA = Constants.ShooterConstants.kA;
            globalConfigs.Slot0.kP = newKP;
            globalConfigs.Slot0.kD = Constants.ShooterConstants.d_Value;

            feedMotor.getConfigurator().apply(globalConfigs);

            lastKP = newKP;
      }
    
    } 


}
