// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.nio.channels.WritableByteChannel;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.*;
import frc.robot.Constants.LimelightConstants;
import frc.robot.commands.*;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    //Set up subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Climber m_Climber = new Climber();

    public final Feeder m_Feeder = new Feeder();
    
    public final Hood m_Hood = new Hood();

    public final Indexer m_Indexer = new Indexer();

    public final Intake m_Intake = new Intake();

    public final LED m_LED = new LED();

    public final Limelight m_Limelight = new Limelight();

    public final Roller m_Roller = new Roller();

    public final Shooter m_Shooter = new Shooter();

    //Set up choosers
    private final SendableChooser<Command> autoChooser2;
    private final SendableChooser<Integer> limelightPipelineChooser;


    public RobotContainer(){
        configureNamedCommands();

        // For convenience a programmer could change this when going to competition.
        // Used to remove testing autos
        boolean isCompetition = true;

        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        autoChooser2 = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("Comp"))
            : stream
        );
        
        SmartDashboard.putData("Auto Chooser", autoChooser2); 

        limelightPipelineChooser = new SendableChooser<>();

        //Foward 0.06 Right 0.249 Up 0.461 Pitch 25
        limelightPipelineChooser.setDefaultOption("Shop Day", 0);
        limelightPipelineChooser.addOption("Shop Night", 1);
        limelightPipelineChooser.addOption("Davis", 2);
        limelightPipelineChooser.addOption("Calibrate", 3);
        limelightPipelineChooser.addOption("Sac", 4);
        limelightPipelineChooser.addOption("Contra", 6);

        SmartDashboard.putData("Pipeline Chooser", limelightPipelineChooser);

        configureBindings();
    }

    //Named commands section
    private void configureNamedCommands(){
        //Pathplanner Auto Commands
        NamedCommands.registerCommand(
            "Main Shoot", 
            Commands.parallel(
                //Part 1
                //Two parts: run shooter and run hood depending on where the robot is facing

                //Should have been the newer autoalign but not tested in auto yet
                new AutoalignRotate(m_Limelight, drivetrain,MaxAngularRate).until(m_Limelight::isCentered).withTimeout(0.5).alongWith(hubShoot()),
                
                //Part 2
                //Add 1 Second delay of cmd group 2
                Commands.waitSeconds(1.5)
                .andThen(BBrestOfShoot())
            )
        );
        NamedCommands.registerCommand(
            "Prepare shoot", 
            m_Shooter.PIDtreeRunMotors().alongWith(m_Indexer.setSpeed(-45).withTimeout(0.1))
        );

        //Intake
        NamedCommands.registerCommand(
            "Deploy Intake", 
            Commands.parallel(
                m_Intake.runOnce(() -> {m_Intake.toSetpoint(0);}),
                m_Roller.setSpeed(90)
            )
        );

        NamedCommands.registerCommand(
            "Retract Intake", 
            m_Intake.runOnce(() -> {m_Intake.toSetpoint(1);})
        );

        //Stop everything
        NamedCommands.registerCommand(
            "Stop everything", 
            Commands.parallel(
            //m_Intake.runOnce(() -> {m_Intake.toSetpoint(1);}),
            m_Indexer.setSpeed(0),
            //m_Shooter.stopMotors(),
            m_Hood.toSetpoint(0),
            //m_Roller.setSpeed(0),
            m_Feeder.setSpeed(0) 
            ).withTimeout(0.1)
        );

        //Raise climber
        NamedCommands.registerCommand("Setup Climb", 
            m_Climber.toSetpoint(1)
        );

        //Lower climber
        NamedCommands.registerCommand("Lower Climber", 
            m_Climber.toSetpoint(2)
        );

        //Auto align
        NamedCommands.registerCommand(
            "Auto Rotate", 
            new AutoalignRotate(m_Limelight, drivetrain, MaxAngularRate)
        );

        //Checks alliance and flips
        NamedCommands.registerCommand(
            "180 flip", 
            drivetrain.runOnce(
                () -> {drivetrain.checkPerspective();}
            )
        );

        //Path finder (never used)
        NamedCommands.registerCommand("Move to Depot", drivetrain.pathfind_test("Move to Depot"));
        NamedCommands.registerCommand("neutral zone", drivetrain.pathfind_test("neutral zone"));
        NamedCommands.registerCommand("Half of neutral zone", drivetrain.pathfind_test("Half of neutral zone"));
        NamedCommands.registerCommand("Move back from hub", drivetrain.pathfind_test("Move back from hub"));
        NamedCommands.registerCommand("top climb", drivetrain.pathfind_test("top climb"));
        NamedCommands.registerCommand("Closer Half of neutral zone", drivetrain.pathfind_test("Closer Half of neutral zone"));
        NamedCommands.registerCommand("Closer Bottom Half of neutral zone", drivetrain.pathfind_test("Closer Bottom Half of neutral zone"));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        //Stops the listed motors when not used
        m_Feeder.setDefaultCommand(
            m_Feeder.setSpeed(0)
        );
        m_Hood.setDefaultCommand(
            m_Hood.toSetpoint(0)
            .andThen(m_Hood.runOnce(() -> {m_Hood.getPosition();}))
            //m_Hood.runOnce(() -> {m_Hood.getPosition();})
        );
        m_Indexer.setDefaultCommand(
            m_Indexer.setSpeed(0)
        );
        m_LED.setDefaultCommand(
            m_LED.run(
                ()-> {
                    m_LED.showTagStatus(m_Limelight.checkForTarget());
                }
            )
        );
        m_Roller.setDefaultCommand(
            m_Roller.setSpeed(0)
        );
        m_Shooter.setDefaultCommand(
            //does the same thing as setSpeed
            m_Shooter.stopMotors()
        );
        m_Intake.setDefaultCommand(
            m_Intake.runOnce(() -> m_Intake.getPosition())
        );
        m_Limelight.setDefaultCommand(
            m_Limelight.runOnce(() -> {m_Limelight.getDistToNearestTag();})
        );

        m_Climber.setDefaultCommand(
            //m_Climber.stopMotors()
            m_Climber.runOnce(()-> m_Climber.getPosition())
        );

        drivetrain.registerTelemetry(logger::telemeterize); 

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        /* Main Controls */

        // Reset the field-centric heading on back button press.
        joystick.back().onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric)
        );

        //testMode();
        shootCtrl(); //96
        intakeCtrl();
        //sysIDCtrl();
        //climbCtrl();
        //hoodCtrl();

        //Manual unjam
        joystick.y().whileTrue(
            m_Indexer.setSpeed(-45)
        );
      
        /* 
        //Test
        joystick.rightBumper().whileTrue(
            Commands.parallel(
                //Part 1
                //Two parts: run shooter and run hood depending on where the robot is facing
                //run shooter based on distance
                new PoseAutoAlign(drivetrain).withTimeout(0.5).alongWith(testShoot()),
                //Part 2
                //Add 1 Second delay of cmd group 2
                m_Indexer.setSpeed(-45).withTimeout(0.1)
                .andThen(Commands.waitSeconds(1.5))
                .andThen(BBrestOfShoot())
            )
        );
        */
    }

    public void shootCtrl(){
        //Hub shoot
        joystick.rightTrigger().whileTrue(
            //Two parts: shooter & hood and everything else
            Commands.parallel(
                //Part 1
                //Two parts: run shooter and run hood depending on where the robot is facing
                //run shooter based on distance
                new PoseAutoAlign(drivetrain).withTimeout(0.3).alongWith(hubShoot()),
                //Part 2
                //Add 1 Second delay of cmd group 2
                m_Indexer.setSpeed(-45).withTimeout(0.1)
                .andThen(Commands.waitSeconds(1.5))
                .andThen(BBrestOfShoot())
            )
        );

         //Ferry Mode
        joystick.rightBumper().whileTrue(
            Commands.parallel(
                //Part 1
                //Two parts: run shooter and run hood depending on where the robot is facing
                Commands.parallel(
                    //run shooter based on distance
                    ferryShoot()
                ),
                //Part 2
                //Add 1 Second delay of cmd group 2
                Commands.waitSeconds(1)
                .andThen(
                    BBrestOfShoot()
                )
            )
        );

    }

    //When hub is inactive or at neutral zone
    public Command ferryShoot(){
        return m_Shooter.PIDrunMotors(25.5).alongWith(m_Hood.toSetpoint(1));
    }

    //Hub is active
    public Command hubShoot(){
        return m_Shooter.PIDtreeRunMotors();
        /*m_Shooter.PIDrunMotors(27.5).withTimeout(0.25)
                .andThen(m_Shooter.PIDtreeRunMotors());*/
    }
    
    /* 
//run shooter based on distance
    public Command mainShoot(){
        return new ConditionalCommand(
                //On true, ferry mode
                m_Shooter.PIDrunMotors(25.5).alongWith(m_Hood.toSetpoint(1)), //Will change
                //On false, hub mode
                m_Shooter.PIDtreeRunMotors(m_Limelight.getDistToNearestTag()).alongWith(m_Hood.run(() -> {m_Hood.getPosition();}))//m_Hood.run(() -> {m_Hood.goToPostion(m_Limelight.getDistToNearestTag());})
                .andThen(new AutoalignRotate(m_Limelight, drivetrain,MaxAngularRate)),
                //Conditional: check if robot is facing drivers
                drivetrain::facingDriver
            );
    }*/
    
    //Two parts: index & feeder
    //run both feeder and indexer

    //Old version(not used at comp)
   public Command restOfShoot(){
        return Commands.parallel(
            m_Feeder.setSpeed(90),
            m_Indexer.setSpeed(80),
            m_Roller.setSpeed(80),
            Commands.waitSeconds(1.5)
            .andThen(
                    Commands.repeatingSequence(
                    m_Intake.retractIntake()
                    .andThen(Commands.waitSeconds(0.5))
                    .andThen(m_Intake.extendIntake())
                    .andThen(Commands.waitSeconds(0.5))
                )
                //m_Intake.slowRetract().until(m_Intake::atEnd)
            )
        );
   }

   //Used in comp! - Giovan
   public Command BBrestOfShoot(){
        return Commands.parallel(
            m_Feeder.PIDtreeRunMotors(),
            m_Indexer.BBtestMotors(),
            m_Roller.setSpeed(80),
            Commands.waitSeconds(1.5)
                .andThen(
                     Commands.repeatingSequence(
                        m_Intake.retractIntake()
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(m_Intake.extendIntake())
                        .andThen(Commands.waitSeconds(0.5))
                    )
                    //m_Intake.slowRetract().until(m_Intake::atEnd)
                )
        );
   }

   //Used to get starter PID values - Giovan
   public void sysIDCtrl(){
    /* Joystick B = Quasistatic forward
        Joystick X = Quasistatic reverse */
        
        //Run tests for about 10 seconds
        joystick.y().whileTrue(
            Commands.parallel(
                m_Shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                m_Feeder.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                m_Indexer.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
            )
        );
        joystick.a().whileTrue(
            Commands.parallel(
                m_Shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                m_Feeder.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                m_Indexer.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
            )        
        );
        joystick.povUp().whileTrue(
            Commands.parallel(
                m_Shooter.sysIdDynamic(SysIdRoutine.Direction.kForward),
                m_Feeder.sysIdDynamic(SysIdRoutine.Direction.kForward),
                m_Indexer.sysIdDynamic(SysIdRoutine.Direction.kForward)
            )        
        );
        joystick.povDown().whileTrue(
            Commands.parallel(
                m_Shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse),
                m_Feeder.sysIdDynamic(SysIdRoutine.Direction.kReverse),
                m_Indexer.sysIdDynamic(SysIdRoutine.Direction.kReverse)
            )        
        );

        //Runs & stops signal logger
        joystick.povLeft().onTrue(
            Commands.runOnce(SignalLogger::start)
        );
        joystick.povRight().onTrue(
            Commands.runOnce(SignalLogger::stop)
        );
   }


   public void hoodCtrl(){
        //Test hood
        joystick.a().whileTrue(
            m_Hood.runEnd(
                () -> {m_Hood.setHood(-5);},
                () -> {m_Hood.setHood(0);}
            )
        );

        //Test hood
        joystick.b().whileTrue(
            m_Hood.runEnd(
                () -> {m_Hood.setHood(5);},
                () -> {m_Hood.setHood(0);}
            )
        );
   }

    //Run climb 
   public void climbCtrl(){
        
        joystick.povLeft().whileTrue(
            m_Climber.toSetpoint(0)
        );
                
        //unclimb after auto
        joystick.povRight().and(joystick.b()).onTrue(
            m_Climber.toSetpoint(1).until(m_Climber::readytoRest)
        );
            /*
            Commands.sequence(
                m_Climber.toSetpoint(1).until(m_Climber::readytoRest)
            .andThen(
                drivetrain.runOnce(
                    () -> {drivetrain.setControl(drive.withVelocityX(1));}
                )
            )
            .andThen(Commands.waitSeconds(0.5))
            .andThen(drivetrain.runOnce(
                    () -> {drivetrain.setControl(drive.withVelocityX(0));}
                )
            )
            .andThen(m_Climber.toSetpoint(0))
            ).withTimeout(2.5)
            );
            */

            /*Commands.sequence(
            m_Climber.toSetpoint(1)//.until(m_Climber::readytoClimb)
            .andThen(Commands.waitSeconds(0.25))
            //.andThen(m_Climber.toSetpoint(2))
            )*/
        
        joystick.povUp().whileTrue(
            m_Climber.toSetpoint(1)
        );
        joystick.povDown().whileTrue(
            m_Climber.toSetpoint(2)

            //Move down from tower and rests climber
            //new unclimb(drivetrain, m_Climber)
        );
   }

   public void intakeCtrl(){
        //Extends intake
        joystick.leftTrigger(0.05).whileTrue(
            Commands.parallel(
                m_Intake.run(
                    () -> {m_Intake.toSetpoint(0);}
                ),
                m_Roller.setSpeed(80)
            )
        );
        
        //Backup intake control
        //Was povUp (might change later)
        joystick.x().whileTrue(
            m_Intake.runEnd(
                () -> {m_Intake.setIntake(90);}, 
                () -> {m_Intake.setIntake(0);}
            )
        );

        /*joystick.povRight().and(joystick.x()).onTrue(
            m_Intake.runOnce(
                () -> {m_Intake.setPosition(-3.219970703125);}
            )
        );*/
   }

   //Not used in comp
    //Experimental shooting stuff
   public void testMode(){
        //Hub shoot
        joystick.rightTrigger().whileTrue(
            //Two parts: shooter & hood and everything else
            Commands.parallel(
                //Part 1
                //Two parts: run shooter and run hood depending on where the robot is facing
                //run shooter based on distance
                Commands.parallel(
                    new PoseAutoAlign(drivetrain).withTimeout(0.3),
                    testShoot()
                ),
                //Part 2
                //Add 1 Second delay of cmd group 2
                m_Indexer.setSpeed(-45).withTimeout(0.1)
                .andThen(Commands.waitSeconds(1.5))
                .andThen(testRestOfShoot())
            )
        );
   }

   //Not used in comp
    public Command testShoot(){
        return m_Shooter.PIDtestRunMotors();
    }

    public Command testRestOfShoot(){
        return Commands.parallel(
            m_Feeder.PIDtreeRunMotors(),
            m_Indexer.PIDrunMotors(),
            m_Roller.setSpeed(80),
            Commands.waitSeconds(1.5)
                .andThen(
                     Commands.repeatingSequence(
                        m_Intake.retractIntake()
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(m_Intake.extendIntake())
                        .andThen(Commands.waitSeconds(0.5))
                    )
                    //m_Intake.slowRetract().until(m_Intake::atEnd)
                )
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser2.getSelected();
    }
    public int getPipeline() {
        return limelightPipelineChooser.getSelected();
    }
}
