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
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagPoses;
import frc.robot.Constants.CameraPoses;

public class Vision extends SubsystemBase {

  //Variables for all the cameras
  private PhotonCamera piCam = new PhotonCamera("piCam");

  //Variables to keep track of stuff
  private final int runningAverageSize = 15;
  private Queue<Double> RAPoseX = new LinkedList<Double>();
  private Queue<Double> RAPoseY = new LinkedList<Double>();
  private Queue<Double> RAPoseSin = new LinkedList<Double>();
  private Queue<Double> RAPoseCos = new LinkedList<Double>();
  private int numberOfTargets = 0;
  private boolean hasValue = false;
  private double poseX = 0;
  private double poseY = 0;
  private Rotation2d poseRotation = new Rotation2d();


  /** Creates a new Vision. */
  public Vision() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

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
          Transform3d robotToCameraPose = CameraPoses.getCameraPose(); //Get Camera's position relative to tag
          Pose3d robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTargetPose, tagPose, robotToCameraPose); //Get robot's position on field
          Pose2d robotPose2d = robotPose3d.toPose2d();

          //Check for major differences between last accepted position
          if(Math.abs(robotPose2d.getY()-poseY) > 2){
            continue; //Y pose is too different. Ignore
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
    }
  }

  /**
   * Update the robot's position based on camera info
   * @param driveTrain
   * @return if it was successful
   */
  public boolean updateOdomentry(DriveTrain driveTrain){
    if (numberOfTargets >= 1) {

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
}
