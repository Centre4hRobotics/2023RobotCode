// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class UpdateMovingOdometry extends CommandBase {
  private DriveTrain _driveTrain;
  private Vision _vision;
  
  //Data stuff
  int totalIterations = 15;
  int index;
  double previousAngle;
  double[] xValues = new double[totalIterations];
  double[] yValues = new double[totalIterations];
  double[] cosValues = new double[totalIterations];
  double[] sinValues = new double[totalIterations];


  /** Creates a new UpdateMovingOdometry. */
  public UpdateMovingOdometry(DriveTrain driveTrain, Vision vision) {
    _driveTrain = driveTrain;
    _vision = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrain, _vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    index = 0;
    previousAngle = _driveTrain.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Get robot info
    DifferentialDriveWheelSpeeds wheelSpeeds = _driveTrain.getWheelSpeeds();
    double averageDriveVelocity = (wheelSpeeds.leftMetersPerSecond + wheelSpeeds.rightMetersPerSecond)/2.;
    double currentAngle = _driveTrain.getAngle();
    double angleVelocity = (currentAngle-previousAngle)/.02;
    previousAngle = currentAngle;

    //Update previous positions based on how the robot has traveled
    for(int i = index-1; i>=0; i--) {
      xValues[i] = xValues[i] + averageDriveVelocity*cosValues[i]*0.02;
      yValues[i] = yValues[i] + averageDriveVelocity*sinValues[i]*0.02;

      double newRotation = Math.atan2(sinValues[i], cosValues[i]) + angleVelocity/180*3.1415*0.02;
      cosValues[i] = Math.cos(newRotation);
      sinValues[i] = Math.sin(newRotation);
    }



    //Get new caemra info
    Pose2d visionPose = _vision.getRawRobotPose();
    if(visionPose != null) {
      Rotation2d CameraRotation = visionPose.getRotation();
      double latency = _vision.getCameraLatency();

      //Get current pose based on latency
      xValues[index] = visionPose.getX() + averageDriveVelocity*CameraRotation.getCos()*latency;
      yValues[index] = visionPose.getY() + averageDriveVelocity*CameraRotation.getSin()*latency;

      //Get current angle based on latency
      double latencyRotation = (CameraRotation.getDegrees() + angleVelocity*latency)/180*3.14159;
      cosValues[index] = Math.cos(latencyRotation);
      sinValues[index] = Math.sin(latencyRotation);

      index++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double bestX = getMedian(xValues);
    double bestY = getMedian(yValues);
    double bestCos = getMedian(cosValues);
    double bestSin = getMedian(sinValues);

    Pose2d bestPose = new Pose2d(bestX, bestY, new Rotation2d(bestCos, bestSin));
    _driveTrain.resetOdometry(bestPose);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return index == totalIterations;
  }

  /**
   * Find the median of a running average data set
   * @param runningAverage a Queue<Double> for the running average
   * @return the median (Double) or Double.NaN if there are no good values
   */
  private Double getMedian(double[] data){
    ArrayList<Double> ordered = new ArrayList<Double>();
    int goodTags = 0;

    //Sort all the good values into ordered
    for (double value : data){

      goodTags ++;
      //Sort value into ordered
      for(int i = 0; i < data.length; i++){
        if(value < data[i]) {
          ordered.add(i, value);
          break;
        }
      }

      //Make sure the value was added
      if (ordered.size() != goodTags){
        ordered.add(value);
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
