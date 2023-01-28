// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

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
  private PhotonCamera piCam = new PhotonCamera("Camera name");

  //Variables to keep track of stuff
  private double poseX = 0;
  private double poseY = 0;
  private Rotation2d rotation = null;
  private int numberOfTargets = 0;

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
      ArrayList<Pose2d> allPose2ds = new ArrayList<Pose2d>();
      List<PhotonTrackedTarget> targets = piCamInfo.getTargets();
      for (PhotonTrackedTarget t : targets) {
        //Get info from tag
        int id_number = t.getFiducialId();  //Apriltag ID
        Transform3d cameraToTargetPose = t.getBestCameraToTarget();

        //Calculate robot's pose on field
        Pose3d tagPose = AprilTagPoses.getPose(id_number);  //Get tag's position on field
        if(tagPose == null){
          continue; //Bad tag, so go to next loop
        }
        Transform3d robotToCameraPose = CameraPoses.getCameraPose(); //Get Camera's position relative to tag
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTargetPose, tagPose, robotToCameraPose); //Get robot's position on field
        allPose2ds.add(robotPose.toPose2d());
      }

      numberOfTargets = allPose2ds.size();
      if (numberOfTargets == 1){
        Pose2d tagPose = allPose2ds.get(0);
        poseX = tagPose.getX();
        poseY = tagPose.getY();
        rotation = tagPose.getRotation();
      } else if (numberOfTargets > 1){
        //Get average rotation and x,y
        double totalX = 0;
        double totalY = 0;
        double totalSin = 0;
        double totalCos = 0;
        for(Pose2d pose : allPose2ds){
          totalX += pose.getX();
          totalY += pose.getY();
          totalSin += pose.getRotation().getSin();
          totalCos += pose.getRotation().getCos();
        }

        poseX = totalX / numberOfTargets;
        poseY = totalY / numberOfTargets;
        double averageSin = totalSin / numberOfTargets;
        double averageCos = totalCos / numberOfTargets;
        rotation = new Rotation2d(averageCos, averageSin);
      }
    }

    //Update network tables
    //Get network table
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    //Write values to network table
    nt.getTable("Vison").getEntry("Number of Tags").setValue(numberOfTargets);
    if(numberOfTargets > 0){
      nt.getTable("Vision").getEntry("X").setValue(poseX);
      nt.getTable("Vision").getEntry("Y").setValue(poseY);
      nt.getTable("Vision").getEntry("Rotation").setValue(rotation);
    } else {
      nt.getTable("Vision").getEntry("X").setValue(null);
      nt.getTable("Vision").getEntry("Y").setValue(null);
      nt.getTable("Vision").getEntry("Rotation").setValue(null);
    }
  }

  //Update the robot's odometry based on feedback from the camera
  public void updateOdomentry(DriveTrain driveTrain){
    Pose2d pose = new Pose2d(poseX, poseY, rotation);
    driveTrain.resetOdometry(pose);
  }
}
