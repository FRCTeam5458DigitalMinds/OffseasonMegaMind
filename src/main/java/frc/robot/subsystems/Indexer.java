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

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volt;


public class Indexer extends SubsystemBase {

    //Initializes the Indexer Motor
    TalonFX indexerMotor;

    //Bang bang setup
    private BangBangController controller = new BangBangController();

    Double testRPS;

    //SysID setup
    private final SysIdRoutine m_sysIdRoutine;
    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    //Feedfoward setup
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.IndexerConstants.kS,Constants.IndexerConstants.kV,Constants.IndexerConstants.kA);

    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);


    private double lastKP = -1;


    public Indexer() {

        //Sets up the settings for the Indexer Motor
        indexerMotor = new TalonFX(Constants.IndexerConstants.indexMotor);
        TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();
        indexerConfigs.Slot0.kS = Constants.IndexerConstants.kS;
        indexerConfigs.Slot0.kV = Constants.IndexerConstants.kV;
        indexerConfigs.Slot0.kA = Constants.IndexerConstants.kA;
        indexerConfigs.Slot0.kP = Constants.IndexerConstants.p_Value;
        
        indexerConfigs.CurrentLimits.withStatorCurrentLimit(65);
        indexerConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);

        indexerMotor.getConfigurator().apply(indexerConfigs);

        indexerMotor.setNeutralMode(NeutralModeValue.Coast); 

        SmartDashboard.putNumber("Index Test RPS", 76);

        //Get kV,kS,kA,kP values
        m_sysIdRoutine = new SysIdRoutine(
              new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
                  Volt.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
          null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("Index state", state.toString())
        ),
          new SysIdRoutine.Mechanism(
          (volts) -> indexerMotor.setControl(m_voltReq.withOutput(volts.in(Volt))),
          null,
          this
        )
      );

      SmartDashboard.putNumber("Index P_value", Constants.IndexerConstants.p_Value);

    }


    //SysID cmds
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.dynamic(direction);
    }

    //Sets the speed for the Indexer Motor
    public Command setSpeed(double OutputPercent){
        return run(
            () -> {
                setIndexer(OutputPercent);
            }
        );
    }

    //Unused - Giovan
    public Command PIDrunMotors(){
        return run(
            () -> {
              setTargetRPS(testRPS);
            }
        );
    }

    //Used - Giovan
    //Bang Bang: testRPS
    public Command BBtestMotors(){
      return run(
        () -> {
          BBrps(testRPS);
        }
      );
    } 

    //Used - Giovan
    //Bang Bang
    public void BBrps(double RPS){
        indexerMotor.setVoltage(controller.calculate(indexerMotor.getVelocity().getValueAsDouble(), RPS)*12 + 0.9*feedforward.calculate(RPS));
    }


    //Uses PID
    public void setTargetRPS(double RPS){
      indexerMotor.setControl(m_request.withVelocity(RPS));
    }

    //Old seting speed
    public void setIndexer(double OutputPercent)
    {
        OutputPercent /= 100.0;
        indexerMotor.set(OutputPercent);
    }

    //Never used
    public Command unjam(){
        return setSpeed(90).withTimeout(0.01) //90
            .andThen(Commands.waitSeconds(1.25))
            .andThen(setSpeed(-90).withTimeout(0.01))
            .andThen(Commands.waitSeconds(0.25))
            ;
    }

     //Continuously runs
    @Override
    public void periodic() {

      //Pulling current value to the Index RPS slider, Used - Giovan
      testRPS = SmartDashboard.getNumber("Index Test RPS", 76);

      //Current RPS
      SmartDashboard.putNumber("Index RPS", indexerMotor.getVelocity().getValueAsDouble());
    
      //Gets the new P value from the elastic slider - Giovan
      double newKP = SmartDashboard.getNumber("Index P_value", Constants.ShooterConstants.p_Value);

      //Comparing old & new values, and updates motors if the new value is different - Giovan
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

            indexerMotor.getConfigurator().apply(globalConfigs);

            lastKP = newKP;
      }
      
    
    } 
}
