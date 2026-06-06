package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;

public class Limelight extends SubsystemBase{
   //SwerveDrive class
   CommandSwerveDrivetrain drivetrain;
   SwerveRequest.RobotCentric robotDrive;
   
   //name of limelight IDs (don't forget)
   String dmllName = Constants.LimelightConstants.ll_Name;
   
   //info from tag
   int id;

   private final int[] redTags = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
   private final int[] blueTags = {17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};

//pose thingys
   private LimelightHelpers.PoseEstimate limelightMeasurement = new LimelightHelpers.PoseEstimate();

   public Limelight(){

    //Setups the limelight position on the robot - Giovan
    LimelightHelpers.setCameraPose_RobotSpace(dmllName, 0.06, 0.249, 0.461, 0, 25, 0);
   }

   //Change smartdashboard to elastic later
    // Basic targeting data
   public Double getTX(){
      //Take tx of current tag
      //double tx = LimelightHelpers.getTX(dmllName);
      //SmartDashboard.putNumber("TX", tx);
      return LimelightHelpers.getTX(dmllName); // Horizontal offset from crosshair to target in degrees

   }

   //just up and down off of the crosshair
   public Double getTY(){

      double ty = LimelightHelpers.getTY(dmllName);
      SmartDashboard.putNumber("TY", ty);
      return ty; // Vertical offset from crosshair to target in degrees
   }

   //Output for TA (Target Area)
   public Double getTA(){
      return LimelightHelpers.getTA(dmllName);
   }

   //Target stuff 
   public boolean checkForTarget(){
      return LimelightHelpers.getTV(dmllName); // Do you have a valid target?
   }

   //current id for the Dashboard
   public Double getTargetID(){

      Double dTarget = Double.parseDouble("0");

      //return the ID
      dTarget = LimelightHelpers.getFiducialID(dmllName);
      SmartDashboard.putNumber("LL-ID", dTarget);
      return dTarget;
   }
   
   //Filters tag
      public int priorityTag(){
        RawFiducial[] currentFiducials = getFiducialData(); //this is the raw data from the limelight
        List<Integer> alCurrentTargetsIDs = new ArrayList<>(); //sets up list of IDs for the field

        for (RawFiducial fiducial : currentFiducials) {//moves raw fiducial ids to target ids
            alCurrentTargetsIDs.add(fiducial.id);
        }
        

        //Red alliance
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){

            if (alCurrentTargetsIDs.contains(5) && alCurrentTargetsIDs.contains(8) && alCurrentTargetsIDs.contains(9) && alCurrentTargetsIDs.contains(10)){
            return 9;
            }
            else if (alCurrentTargetsIDs.contains(2) && alCurrentTargetsIDs.contains(9) && alCurrentTargetsIDs.contains(10) && alCurrentTargetsIDs.contains(11)){
                return 11;
            }
            else if (alCurrentTargetsIDs.contains(8) && alCurrentTargetsIDs.contains(9) && alCurrentTargetsIDs.contains(10)){
                return 9;
            }
            else if (alCurrentTargetsIDs.contains(10) && alCurrentTargetsIDs.contains(11) && alCurrentTargetsIDs.contains(2)){
                return 11;
            }
            else if (alCurrentTargetsIDs.contains(9) && alCurrentTargetsIDs.contains(10) && alCurrentTargetsIDs.contains(11)){
                return 10;
            }
            else if (alCurrentTargetsIDs.contains(5) && alCurrentTargetsIDs.contains(8)){
                return 8;
            }
            else if (alCurrentTargetsIDs.contains(9) && alCurrentTargetsIDs.contains(10)){
                return 10;
            }
            else if (alCurrentTargetsIDs.contains(11) && alCurrentTargetsIDs.contains(2)){
                return 11;
            }
            else if (alCurrentTargetsIDs.contains(10)){
                return 10;
            }
            else if (alCurrentTargetsIDs.isEmpty()){
                return 0;
            } 
            else{
                return alCurrentTargetsIDs.get(0);
            }

        }//Blue alliance
        else
        {
            
            if (alCurrentTargetsIDs.contains(21) && alCurrentTargetsIDs.contains(24) && alCurrentTargetsIDs.contains(25) && alCurrentTargetsIDs.contains(26)){
                return 25;
            }
            else if (alCurrentTargetsIDs.contains(18) && alCurrentTargetsIDs.contains(25) && alCurrentTargetsIDs.contains(26) && alCurrentTargetsIDs.contains(27)){
                return 27;
            }
            else if (alCurrentTargetsIDs.contains(24) && alCurrentTargetsIDs.contains(25) && alCurrentTargetsIDs.contains(26)){
                return 25;
            }
            else if (alCurrentTargetsIDs.contains(26) && alCurrentTargetsIDs.contains(27) && alCurrentTargetsIDs.contains(18)){
                return 27;
            }
            else if (alCurrentTargetsIDs.contains(25) && alCurrentTargetsIDs.contains(26) && alCurrentTargetsIDs.contains(27)){
                return 26;
            }
            else if (alCurrentTargetsIDs.contains(21) && alCurrentTargetsIDs.contains(24)){
                return 24;
            }
            else if (alCurrentTargetsIDs.contains(25) && alCurrentTargetsIDs.contains(26)){
                return 26;
            }
            else if (alCurrentTargetsIDs.contains(27) && alCurrentTargetsIDs.contains(18)){
                return 27;
            }
            else if (alCurrentTargetsIDs.contains(26)){
                return 26;
            }
            else if (alCurrentTargetsIDs.isEmpty()){
                return 0;
            } 
            else{
                return alCurrentTargetsIDs.get(0);
            }
        }

        //Note: add red field tags
        //groups tags & picks a tag
          
   }

   //Get distance from tag
   public Double getDistToNearestTag(){

    double distance = 2.2; //Default distance if no tag
    double halfofBumper = 0.87/2; //Distance from center to edge of bumper
        
    RawFiducial[] crtFiducials = getFiducialData();

      //moves raw fiducial to current target ids, takes only tx of the current target id    
         for (RawFiducial fiducial : crtFiducials) {

                if (fiducial.id == priorityTag()){
                    distance = fiducial.distToRobot; // X offset (no crosshair)
               }
         }
      SmartDashboard.putNumber("Dist to Robot", distance - halfofBumper);
      
      //Distance from tag to front of bumper
      return distance;
   }

   

   //Gives more consistant shooter distance (not used anymore)
    private double filteredShooterDistance = 2.2;
    private boolean hasValidShooterDistance = false;

    public void updateShooterDistance() {

        double halfofBumper = 0.87/2; //Distance from center to edge of bumper

        RawFiducial[] currentFiducials = getFiducialData();
        int targetId = priorityTag();

        for (RawFiducial fiducial : currentFiducials) {
            if (fiducial.id == targetId) {
                double measuredDistance = fiducial.distToRobot - halfofBumper;

                if (!hasValidShooterDistance) {
                    filteredShooterDistance = measuredDistance;
                } else {
                    double delta = measuredDistance - filteredShooterDistance;

                    // Reject big single-frame jumps
                    if (Math.abs(delta) < 0.35) {
                        filteredShooterDistance =
                            filteredShooterDistance
                            + 0.25 * delta;
                    }
                }

                hasValidShooterDistance = true;
                SmartDashboard.putNumber("Shooter Raw Distance", measuredDistance);
                SmartDashboard.putNumber("Shooter Filtered Distance", filteredShooterDistance);
                return;
            }
        }

    // Keep last good value if no valid target this frame
    SmartDashboard.putNumber("Shooter Raw Distance", 2.2);
    SmartDashboard.putNumber("Shooter Filtered Distance", filteredShooterDistance);
    }
    
    public double getShooterDistance() {
        return filteredShooterDistance;
    }

    public boolean hasShooterDistance() {
        return hasValidShooterDistance;
    }
   

  public double limelight_aim_proportional(Double robotMaxAngularSpeed, double currentTX){    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double AutoalignkP = 0.01; //0.035

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = currentTX * AutoalignkP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= robotMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;
    
    return targetingAngularVelocity;
  }

  //Get velocity needed to move to certain TY
  public double limelight_range_proportional(Double robotMaxLinearSpeed){    
    double AutoalignkP = 1.0; 

    double targetingLinearVelocity = getTY() * AutoalignkP;

    // convert to radians per second for our drive method
    targetingLinearVelocity *= robotMaxLinearSpeed;

    targetingLinearVelocity *= -1.0;
    
    return targetingLinearVelocity;
  }

  //Raw fiducial data
  public RawFiducial[] getFiducialData() {

    // Get raw AprilTag/Fiducial data from the Limelight (assuming default name "limelight")
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(dmllName);

    return fiducials;
  }
  
  //Checks if centered to tag
  public boolean isCentered(){
    /*double txnc = 99; //Assumes not centered if no tag
    RawFiducial[] crtFiducials = getFiducialData();

    //moves raw fiducial to current target ids, takes only tx of the current target id    
    for (RawFiducial fiducial : crtFiducials) {

        if (fiducial.id == priorityTag()){
            txnc = fiducial.txnc; // X offset (no crosshair)
        }
    }*/

    //If txnc is between these numbers, robot is aiming at the hub
    if (-10 < SmartDashboard.getNumber("TXNC", 99) && SmartDashboard.getNumber("TXNC", 99) < -5){ //-23 & -30
        return true;
    }
    
    //If not, assume it is not
    return false;
  }


  //Have limelight only use certain tags based on alliance
  public void setPriorityTags(){
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
        LimelightHelpers.SetFiducialIDFiltersOverride(dmllName, redTags);
    }
    else{

        LimelightHelpers.SetFiducialIDFiltersOverride(dmllName, blueTags);

    }
  }

  @Override
  public void periodic(){

    //Setup Red or Blue only tags
    setPriorityTags();

    //Research on usage - Giovan
    //Old distance from hub
    updateShooterDistance();
  }
}

