// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class ClimbConstants {
    public static final int climbMotor = 60;

      public static final double climb_P = 0.5; //
      public static final double climb_D = 0.0;
  }

  public static class HoodConstants {
    public static final int hoodMotor = 40;

      public static final double hood_P = 0.7;
      public static final double hood_D = 0.0;
  }

  public static class IndexerConstants {
    public static final int indexMotor = 30;

    public static final double kS = 0.15715;
    public static final double kV = 0.11623;
    public static final double kA = 0.0020998;
    public static final double p_Value = 0.027263;



  }
  
  public static class IntakeConstants {
    public static final int intakeMotor = 50;

      public static final double intake_P = 1.0; //1.0
      public static final double intake_D = 0.0;
    
    public static final int rollerMotor = 51;

  }

  public static class LimelightConstants {
    public static final String ll_Name = "limelight-digital";
    public static int pipeline = 2;    /// Night in shope (1), day in shop (0), At citrus (2)...
  }

  public static class ShooterConstants {
    public static final int lowerFlyWheel = 20;
    public static final int upperFlyWheel = 21;
    

    public static final int kraken_RPS = 6000/60;

    public static final double p_Value = 0.4; //0.068124
    public static final double i_Value = 0.0;
    public static final double d_Value = 0.0;

    public static final double kS = 0.22498;
    public static final double kV = 0.12323;
    public static final double kA = 0.013054;

  }
  
  public static class FeederConstants {
    public static final int feeder = 22;

    public static final double kS = 0.13157;
    public static final double kV = 0.11908;
    public static final double kA = 0.0066253;
    public static final double p_Value = 0.216653;

  }

  public static class LEDConstants{
    public static final int PDH = 1;
  }
}
