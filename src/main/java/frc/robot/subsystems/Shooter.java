// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.units.VoltageUnit;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;




public class Shooter extends SubsystemBase {
    
    //Initalizes the motors
    private TalonFX lowerFlyMotor;
    private TalonFX upperFlyMotor;

    //sets up velocity PID for slot 0
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    //Calculates the max Revolutions Per Second
    private final double maxRPM = 5500;


    BangBangController controller = new BangBangController();

    InterpolatingDoubleTreeMap shooterRPS;

    /*Notes:
    10%: 550
    20%  1100
    30%  1650
    40%  2200
    50%  2750
    60%  3300
    70%  3850
    80%  4400
    90%  4950
    100% 5500
    */

    private final SysIdRoutine m_sysIdRoutine;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ShooterConstants.kS,Constants.ShooterConstants.kV,Constants.ShooterConstants.kA);

    Double testRPS;
    Double attemptRPS;

    SysIdRoutine routine;


    double multiplier;
    private double lastKP = -1;
    public Shooter() {

        //m_shooterFeedback.setTolerance(Constants.ShooterConstants.kShooterToleranceRPS);
        
        //Sets settings for the Lower Flywheel
        
        TalonFXConfiguration globalConfigs = new TalonFXConfiguration();
        globalConfigs.CurrentLimits.withStatorCurrentLimit(60);
        globalConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        
        //setups the PID value for the intake
      
        globalConfigs.Slot0.kS = Constants.ShooterConstants.kS;
        globalConfigs.Slot0.kV = Constants.ShooterConstants.kV;
        globalConfigs.Slot0.kA = Constants.ShooterConstants.kA;
        globalConfigs.Slot0.kP = Constants.ShooterConstants.p_Value;
        globalConfigs.Slot0.kD = Constants.ShooterConstants.d_Value;
        
        lowerFlyMotor = new TalonFX(Constants.ShooterConstants.lowerFlyWheel);
        
        lowerFlyMotor.getConfigurator().apply(globalConfigs);

        //Sets settings for Upper Flywheel
        
        upperFlyMotor = new TalonFX(Constants.ShooterConstants.upperFlyWheel);

        upperFlyMotor.getConfigurator().apply(globalConfigs);

        upperFlyMotor.setControl(new Follower(lowerFlyMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        lowerFlyMotor.setNeutralMode(NeutralModeValue.Coast);
        upperFlyMotor.setNeutralMode(NeutralModeValue.Coast);




        /* 
          Context to Interpolating Double Tree Map:
          
            *Creates a plot of data points
            *Uses data points to estimate value based off key
        */

      //5th attempt
      shooterRPS = new InterpolatingDoubleTreeMap();
        //Key: distance in meters
        //Value: velocity of shooter

      /*shooterRPS.put(2.084979071669848,24.55); //close (tune again)
      shooterRPS.put(3.0008923066606354,27.); //Standard
      shooterRPS.put(4.434153557029914,32.5); //Far*/


      shooterRPS.put(1.7,22.58); //close (tune again)
      shooterRPS.put( 2.889059 ,27.02); //Standard
      shooterRPS.put(3.873 ,30.17); //Far
      // 1.7 22.58 close
      // 2.889059 27.02 Standard
      // 3.873 30.17 Far

      //Default test RPS
      SmartDashboard.putNumber("Shooter Test RPS", 27.02);    
      
      //Default P_value
      SmartDashboard.putNumber("Shooter P_value", Constants.ShooterConstants.p_Value);

      SmartDashboard.putNumber("Multiplier", 0.96);


      //Get kV,kS,kA,kP values
      m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
    null,        // Use default ramp rate (1 V/s)
              Volt.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
      null,        // Use default timeout (10 s)
                  // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("Shooter state", state.toString())
          ),
        new SysIdRoutine.Mechanism(
          (volts) -> lowerFlyMotor.setControl(m_voltReq.withOutput(volts.in(Volt))),
          null,
          this
        )
      );

    }

  
    //No PID or Bang Bang
    
    /*//sets up a command for the speed of the motors
    public Command setSpeed(double percent){
      return run(
          () -> {
            setLowerFly(percent);
          }
      );
    }
    //Made seperate because it doesn't use setControl
    public Command stopMotors(){
      return run(
          () -> {
            setLowerFly(0);
          }
      );
    }

    //Sets the speed of the Lower Flywheel
    public void setLowerFly(double OutputPercent)
    {
      OutputPercent /= 100.0;
      lowerFlyMotor.set(OutputPercent);
      SmartDashboard.putNumber("Flywheel speed", OutputPercent);
      //lowerFlyMotor.setControl(m_request.withVelocity(RPS*OutputPercent));
    }*/
    

    //PID: runs desired RPS
    public Command PIDrunMotors(double FlyRPS){
      return run(
          () -> {
            setTargetRPS(FlyRPS);
          }
      );
    }

    //Stop
    public Command stopMotors(){
      return run(
          () -> {
            lowerFlyMotor.set(0);
          }
      );
    }

    //PID: uses distance from hub
    public Command PIDtreeRunMotors(){
      return run(
          () -> {
            setTargetRPS(attemptRPS);
          }
      );
    }

    //PID: test RPS
    public Command PIDtestRunMotors(){
      return run(
          () -> {
            setTargetRPS(testRPS);
          }
      );
    }

    //Bang Bang: test RPS
    public Command BBtestMotors(){
      return run(
        () -> {
          BBrps(testRPS);
        }
      );
    }

    //Just Bang Bang
    public Command BBtreeMotors(double distance){
      return run(
        () -> {
          BBrps(shooterRPS.get(distance));
        }
      );
    }

    //Bang Bang
    public void BBrps(double RPS){
        lowerFlyMotor.setVoltage(controller.calculate(lowerFlyMotor.getVelocity().getValueAsDouble(), RPS)*12 + 0.9*feedforward.calculate(RPS));
    }

    //PID
    public void setTargetRPS(double RPS){
      SmartDashboard.putNumber("RPS", lowerFlyMotor.getVelocity().getValueAsDouble());
      lowerFlyMotor.setControl(m_request.withVelocity(RPS));
    }

    //After week 3 feature
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.dynamic(direction);
    }

    //Continuously runs
   @Override
   public void periodic() {

    //RPS from the interpolating table
    attemptRPS = multiplier*shooterRPS.get(SmartDashboard.getNumber("Pose Distance", 27.02));

    //Desired RPS
    SmartDashboard.putNumber("Attempted Shooter RPS", shooterRPS.get(SmartDashboard.getNumber("Pose Distance", 27.02)));

    //Multiplier
    multiplier = SmartDashboard.getNumber("Multiplier", 0.96);


    //Testing RPS from elastic
    testRPS = SmartDashboard.getNumber("Shooter Test RPS", 27.02);

    //Current RPS
    SmartDashboard.putNumber("Shooter RPS", lowerFlyMotor.getVelocity().getValueAsDouble());

    double newKP = SmartDashboard.getNumber("Shooter P_value", Constants.ShooterConstants.p_Value);

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

            lowerFlyMotor.getConfigurator().apply(globalConfigs);
            upperFlyMotor.getConfigurator().apply(globalConfigs);

            lastKP = newKP;
    }
  }
}
