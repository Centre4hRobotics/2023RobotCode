// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagPoses;
import frc.robot.Constants.CameraPoses;

public class Vision extends SubsystemBase {

  //Variables for all the cameras
  private PhotonCamera piCam = new PhotonCamera("piCam");

  //Variables to keep track of stuff
  private final int runningAverageSize = 15;
  // private Queue<Double> RAPoseX = new LinkedList<Double>();
  // private Queue<Double> RAPoseY = new LinkedList<Double>();
  // private Queue<Double> RAPoseSin = new LinkedList<Double>();
  // private Queue<Double> RAPoseCos = new LinkedList<Double>();
  private ArrayList<Double> RAPoseX = new ArrayList<Double>();
  private ArrayList<Double> RAPoseY = new ArrayList<Double>();
  private ArrayList<Double> RAPoseSin = new ArrayList<Double>();
  private ArrayList<Double> RAPoseCos = new ArrayList<Double>();
  private int numberOfTargets = 0;
  private boolean hasValue = false;
  private double poseX = 0;
  private double poseY = 0;
  private Rotation2d poseRotation = new Rotation2d();
  private final double yTolerance = .8;
  private final double xTolerance = .6;
  private Pose2d lastRawPose;
  private double latency;//Camera latency in seconds
  private long previousTime = 0;
  private double previousAngle = 0;
  private double previousVelocity = 0;

  private boolean _isCompBot;
  private final Arm _arm;
  private final DriveTrain _driveTrain;


  /** Creates a new Vision. */
  public Vision(boolean isCompBot, Arm arm, DriveTrain driveTrain) {
    _isCompBot = isCompBot;
    _arm = arm;
    _driveTrain = driveTrain;
    //piCam.setLED(VisionLEDMode.kOn);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    //Old vision filtering -> also change variables up top!
    /*numberOfTargets = 0;  //Reset how many targets it sees

    PhotonPipelineResult piCamInfo = piCam.getLatestResult(); //Get most recent info from camera
    boolean piCamHasTargets = piCamInfo.hasTargets(); //Checks if it actually sees any targets
    if (piCamHasTargets) {
      
      List<PhotonTrackedTarget> targets = piCamInfo.getTargets();

      for (PhotonTrackedTarget t : targets) {
        //Get info from tag
        int id_number = t.getFiducialId();  //Apriltag ID
        Transform3d cameraToTargetPose = t.getBestCameraToTarget();

        //Calculate robot's pose on field
        Pose3d tagPose = AprilTagPoses.getPose(id_number);  //Get tag's position on field
        if(tagPose == null){
          continue; //Tag isn't on the field. Go to next one
        } else{
          //Get robot's position
          Transform3d robotToCameraPose = CameraPoses.getCameraPose(_arm.isRaised(), _isCompBot); //Get Camera's position relative to tag
          Pose3d robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTargetPose, tagPose, robotToCameraPose); //Get robot's position on field
          Pose2d robotPose2d = robotPose3d.toPose2d();

          //I added this
          lastRawPose = robotPose2d;
          latency = piCamInfo.getLatencyMillis()/1000.0;

          //Check for major differences between last accepted position;  If we are close enough, current pose is accurate and should overrule
          if((Math.abs(robotPose2d.getY()-poseY) > yTolerance || Math.abs(robotPose2d.getX()-poseX) > xTolerance) && Math.abs(poseX-Constants.fieldCenterX)<4){
            //See if alternate position works
            Transform3d AltCameraToTargetPose = t.getAlternateCameraToTarget();
            Transform3d AltRobotToCameraPose = CameraPoses.getCameraPose(_arm.isRaised(), _isCompBot);
            Pose3d AltRobotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(AltCameraToTargetPose, tagPose, AltRobotToCameraPose); //Get robot's position on field
            Pose2d AltRobotPose2d = AltRobotPose3d.toPose2d();

            //See if this recreation is better
            if (Math.abs(AltRobotPose2d.getY()-poseY) < yTolerance && Math.abs(AltRobotPose2d.getX()-poseX) < xTolerance){
              robotPose2d = AltRobotPose2d;

              NetworkTableInstance nt = NetworkTableInstance.getDefault();
              nt.getTable("Vision").getEntry("Alternate Pos Used").setValue(robotPose2d.toString());
            } else{
              NetworkTableInstance nt = NetworkTableInstance.getDefault();
              nt.getTable("Vision").getEntry("Alternate Pos Unused").setValue(AltRobotPose2d.toString());

              continue; //Pose is too different. Ignore
            }
            
          }

          //Update running average
          RAPoseX.add(robotPose2d.getX());
          RAPoseY.add(robotPose2d.getY());
          RAPoseCos.add(robotPose2d.getRotation().getCos());
          RAPoseSin.add(robotPose2d.getRotation().getSin());
          numberOfTargets ++;
        }
      }
    }

    //Add null frame when robot doesn't see anything
    if (numberOfTargets == 0){
      RAPoseX.add(Double.NaN);
      RAPoseY.add(Double.NaN);
      RAPoseSin.add(Double.NaN);
      RAPoseCos.add(Double.NaN);
    }

    //Delete old data to keep running average
    while(RAPoseX.size() > runningAverageSize){
      RAPoseX.remove();
      RAPoseY.remove();
      RAPoseSin.remove();
      RAPoseCos.remove();
    }

    //Get average position of robot
    Double poseX_median = getMedian(RAPoseX);
    if(poseX_median == Double.NaN){
      hasValue = false;
    } else{
      hasValue = true;
      poseX = poseX_median;
      poseY = getMedian(RAPoseY);
      poseRotation = new Rotation2d(getMedian(RAPoseCos), getMedian(RAPoseSin));
    }

    //Update network tables
    //Get network table
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    //Write values to network table
    nt.getTable("Vision").getEntry("Number of Tags").setValue(numberOfTargets);
    if(hasValue){
      nt.getTable("Vision").getEntry("X").setValue(poseX);
      nt.getTable("Vision").getEntry("Y").setValue(poseY);
      nt.getTable("Vision").getEntry("Rotation").setValue(poseRotation.toString());
    } else {
      nt.getTable("Vision").getEntry("X").unpublish();
      nt.getTable("Vision").getEntry("Y").unpublish();
      nt.getTable("Vision").getEntry("Rotation").unpublish();
    }*/


    //New vision filtering
    //Get time info
    long currentTime = System.currentTimeMillis();
    double deltaTime = (currentTime-previousTime)/1000.0;
    previousTime = currentTime;
 
    //Get robot info
    DifferentialDriveWheelSpeeds wheelSpeeds = _driveTrain.getWheelSpeeds();
    double averageDriveVelocity = (wheelSpeeds.leftMetersPerSecond + wheelSpeeds.rightMetersPerSecond)/2.;
    double averageDriveAceleration = (averageDriveVelocity-previousVelocity)/deltaTime;
    previousVelocity = averageDriveVelocity;
    double currentAngle = _driveTrain.getAngle();
    double angleVelocity = (currentAngle-previousAngle)/deltaTime;
    previousAngle = currentAngle;

    //Update previous positions based on how the robot has traveled
    for(int i = 0; i<RAPoseX.size(); i++) {
      RAPoseX.set(i, RAPoseX.get(i) + averageDriveVelocity*RAPoseCos.get(i)*deltaTime + averageDriveAceleration*RAPoseCos.get(i)*deltaTime*deltaTime/2.);
      RAPoseY.set(i, RAPoseY.get(i) + averageDriveVelocity*RAPoseSin.get(i)*deltaTime + averageDriveAceleration*RAPoseSin.get(i)*deltaTime*deltaTime/2.);

      double newRotation = Math.atan2(RAPoseSin.get(i), RAPoseCos.get(i)) + angleVelocity/180*3.1415*deltaTime;
      RAPoseCos.set(i, Math.cos(newRotation));
      RAPoseSin.set(i, Math.sin(newRotation));
    }


    numberOfTargets = 0;  //Reset how many targets it sees

    PhotonPipelineResult piCamInfo = piCam.getLatestResult(); //Get most recent info from camera
    boolean piCamHasTargets = piCamInfo.hasTargets(); //Checks if it actually sees any targets
    if (piCamHasTargets) {
      
      List<PhotonTrackedTarget> targets = piCamInfo.getTargets();

      for (PhotonTrackedTarget t : targets) {
        //Get info from tag
        int id_number = t.getFiducialId();  //Apriltag ID
        Transform3d cameraToTargetPose = t.getBestCameraToTarget();

        //Calculate robot's pose on field
        Pose3d tagPose = AprilTagPoses.getPose(id_number);  //Get tag's position on field
        if(tagPose == null){
          continue; //Tag isn't on the field. Go to next one
        } else{
          //Get robot's position
          Transform3d robotToCameraPose = CameraPoses.getCameraPose(_arm.isRaised(), _isCompBot); //Get Camera's position relative to tag
          Pose3d robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTargetPose, tagPose, robotToCameraPose); //Get robot's position on field
          Pose2d robotPose2d = robotPose3d.toPose2d();

          //I added this
          lastRawPose = robotPose2d;
          Rotation2d CameraRotation = robotPose2d.getRotation();
          latency = piCamInfo.getLatencyMillis()/1000.0;


          //Update running average with just-found data (based on latency)
          RAPoseX.add(0, robotPose2d.getX() + averageDriveVelocity*CameraRotation.getCos()*latency + averageDriveAceleration*CameraRotation.getCos()*latency*latency/2.);
          RAPoseY.add(0, robotPose2d.getY() + averageDriveVelocity*CameraRotation.getSin()*latency + averageDriveAceleration*CameraRotation.getSin()*latency*latency/2.);
          double latencyRotation = (CameraRotation.getDegrees() + angleVelocity*latency)/180*3.14159;
          RAPoseCos.add(0, Math.cos(latencyRotation));
          RAPoseSin.add(0, Math.sin(latencyRotation));
          numberOfTargets ++;
        }
      }
    }

    //Add null frame when robot doesn't see anything
    if (numberOfTargets == 0){
      RAPoseX.add(0, Double.NaN);
      RAPoseY.add(0, Double.NaN);
      RAPoseSin.add(0, Double.NaN);
      RAPoseCos.add(0, Double.NaN);
    }

    //Delete old data to keep running average
    while(RAPoseX.size() > runningAverageSize){
      int lastIndex = RAPoseX.size()-1;
      RAPoseX.remove(lastIndex);
      RAPoseY.remove(lastIndex);
      RAPoseSin.remove(lastIndex);
      RAPoseCos.remove(lastIndex);
    }

    //Get average position of robot
    Double poseX_median = getMedian(RAPoseX);
    if(poseX_median == Double.NaN){
      hasValue = false;
    } else{
      hasValue = true;
      poseX = poseX_median;
      poseY = getMedian(RAPoseY);
      poseRotation = new Rotation2d(getMedian(RAPoseCos), getMedian(RAPoseSin));
    }

    //Update network tables
    //Get network table
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    //Write values to network table
    nt.getTable("Vision").getEntry("Number of Tags").setValue(numberOfTargets);
    if(hasValue){
      nt.getTable("Vision").getEntry("X").setValue(poseX);
      nt.getTable("Vision").getEntry("Y").setValue(poseY);
      nt.getTable("Vision").getEntry("Rotation").setValue(poseRotation.toString());
    } else {
      nt.getTable("Vision").getEntry("X").unpublish();
      nt.getTable("Vision").getEntry("Y").unpublish();
      nt.getTable("Vision").getEntry("Rotation").unpublish();
    }

  }

  /**
   * Update the robot's position based on camera info
   * @param driveTrain
   * @param useReasonable Should we check to make sure vision and drive odometry are similar?
   * @return if it was successful
   */
  public boolean updateOdomentry(DriveTrain driveTrain, boolean useReasonable){
    double xAcceptable = 1;
    double yAcceptable = 1.5;
    if (hasValue) {

      if(useReasonable){
        if (Math.abs(poseX - driveTrain.getPose().getX()) > xAcceptable || Math.abs(poseY - driveTrain.getPose().getY()) > yAcceptable){
          //Vision and drive train do not match, so do not use
          NetworkTableInstance nt = NetworkTableInstance.getDefault();
          nt.getTable("Vision").getEntry("reset").setValue(false);

          return false;
        }
      }

      Pose2d pose = new Pose2d(poseX, poseY, poseRotation);
      driveTrain.resetOdometry(pose);

      NetworkTableInstance nt = NetworkTableInstance.getDefault();
      nt.getTable("Vision").getEntry("reset").setValue(true);

      return true;
    } else {
      NetworkTableInstance nt = NetworkTableInstance.getDefault();
      nt.getTable("Vision").getEntry("reset").setValue(false);

      return false;
    }
  }

  /**
   * Update robot's position based on camera info as long as it sees an Apriltag
   * @param driveTrain
   * @return if it was successful
   */
  public boolean updateOdomentry(DriveTrain driveTrain){
    return updateOdomentry(driveTrain, false);
  }

  /**
   * Get unfiltered robot position based on camera
   * @return calculated robot position or null if no target
   */
  public Pose2d getRawRobotPose() {
    if(numberOfTargets > 0){
      return lastRawPose;
    } else{
      return null;
    }
  }

  /**
   * Get camera latency
   * @return camera latency in seconds
   */
  public double getCameraLatency(){
    return latency;
  }


  /**
   * Find the median of a running average data set
   * @param runningAverage a Queue<Double> for the running average
   * @return the median (Double) or Double.NaN if there are no good values
   */
  private Double getMedian(Queue<Double> runningAverage){
    ArrayList<Double> ordered = new ArrayList<Double>();
    int goodTags = 0;

    //Sort all the good values into ordered
    for (Double value : runningAverage){
      if (value == Double.NaN){
        continue; //Empty frame
      } else{
        //Found another good tag
        goodTags ++;

        //Sort value into ordered
        for(int i = 0; i < ordered.size(); i++){
          if(value < ordered.get(i)) {
            ordered.add(i, value);
            break;
          }
        }

        //Make sure the value was added
        if (ordered.size() != goodTags){
          ordered.add(value);
        }
      }
    }

    //Find median
    if(goodTags == 0){
      return Double.NaN;  //There are no values to find the median of
    } else if(goodTags % 2 == 1){ //There is an odd number of values
      int index = (goodTags - 1)/2;
      return ordered.get(index);
    } else{ //There is an even number of values
      int index1 = goodTags/2;
      int index2 = index1-1;
      return (ordered.get(index1) + ordered.get(index2))/2.0;
    }
  }


  /**
   * Find the median of a running average data set
   * @param runningAverage a Queue<Double> for the running average
   * @return the median (Double) or Double.NaN if there are no good values
   */
  private Double getMedian(ArrayList<Double> runningAverage){
    ArrayList<Double> ordered = new ArrayList<Double>();
    int goodTags = 0;

    //Sort all the good values into ordered
    for (Double value : runningAverage){
      if (value == Double.NaN){
        continue; //Empty frame
      } else{
        //Found another good tag
        goodTags ++;

        //Sort value into ordered
        for(int i = 0; i < ordered.size(); i++){
          if(value < ordered.get(i)) {
            ordered.add(i, value);
            break;
          }
        }

        //Make sure the value was added
        if (ordered.size() != goodTags){
          ordered.add(value);
        }
      }
    }

    //Find median
    if(goodTags == 0){
      return Double.NaN;  //There are no values to find the median of
    } else if(goodTags % 2 == 1){ //There is an odd number of values
      int index = (goodTags - 1)/2;
      return ordered.get(index);
    } else{ //There is an even number of values
      int index1 = goodTags/2;
      int index2 = index1-1;
      return (ordered.get(index1) + ordered.get(index2))/2.0;
    }
  }

  
}
