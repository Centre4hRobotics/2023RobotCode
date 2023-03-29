// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AprilTagPoses;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.FieldSide;
import frc.robot.Constants.Offset;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class AutoPickup extends CommandBase {
  /** Creates a new AutoPickup. */

  private Vision _vision;
  private DriveTrain _driveTrain;
  private Arm _arm;
  private Command _command;
  private Offset _offset;
  private boolean _endEarly;


  public AutoPickup(DriveTrain driveTrain, Arm arm, Vision vision, Offset offset) {
    _vision = vision;
    _driveTrain = driveTrain;
    _arm = arm;
    _offset = offset;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrain, _arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _endEarly = !_vision.updateOdomentry(_driveTrain);

    int bestTag = -1;
    Pose2d robotPose = _driveTrain.getPose();
    double bestHypot = 5;

    //Determine which tag we are closest to
    int[] possibleTags;
    if (robotPose.getY() >= FieldPoses.loadingY) {
      possibleTags = new int[] {4, 5};
    } else if (robotPose.getX() < FieldPoses.fieldWidth / 2) {
      possibleTags = new int[] {6, 7, 8};
    } else {
      possibleTags = new int[] {1, 2, 3};
    }

    for (int i = 1; i <= possibleTags.length; i++) {
      Pose2d tagPose = AprilTagPoses.getPose(possibleTags[i]).toPose2d();
      double hypot = Math.hypot(robotPose.getX()-tagPose.getX(), robotPose.getY()-tagPose.getY());
      if(hypot < bestHypot){
        bestHypot = hypot;
        bestTag = possibleTags[i];
      }
    }

    //Get target pose
    FieldSide side = null;
    int grid = -1;
    int node = -1;
    Pose2d targetPose;

    if (bestTag == -1) {
      _endEarly = true;
      System.out.println("No tag is closer than " + bestHypot + "m");
      targetPose = _driveTrain.getPose();
    } else if(bestTag <= 3) {
      side = FieldSide.RIGHT;
      grid = bestTag-1;
      if(_offset == Offset.RIGHT) {
        try {
          targetPose = FieldPoses.getScoringPose(side, grid, 2);
          node = 2;
        } catch (Exception e) {
          e.printStackTrace();
          targetPose = _driveTrain.getPose();
        }
      }
      else if(_offset == Offset.CENTER) {
        try {
          targetPose = FieldPoses.getScoringPose(side, grid, 1);
          node = 1;
        } catch (Exception e) {
          e.printStackTrace();
          targetPose = _driveTrain.getPose();
        }
      }
      else {
        try {
          targetPose = FieldPoses.getScoringPose(side, grid, 0);
          node = 0;
        } catch (Exception e) {
          e.printStackTrace();
          targetPose = _driveTrain.getPose();
        }
      }

    } else if(bestTag >= 6){
      side = FieldSide.LEFT;
      grid = 8-bestTag;
      if(_offset == Offset.RIGHT) {
        try {
          targetPose = FieldPoses.getScoringPose(side, grid, 0);
          node = 0;
        } catch (Exception e) {
          e.printStackTrace();
          targetPose = _driveTrain.getPose();
        }
      }
      else if(_offset == Offset.CENTER) {
        try {
          targetPose = FieldPoses.getScoringPose(side, grid, 1);
          node = 1;
        } catch (Exception e) {
          e.printStackTrace();
          targetPose = _driveTrain.getPose();
        }
      }
      else {
        try {
          targetPose = FieldPoses.getScoringPose(side, grid, 2);
          node = 2;
        } catch (Exception e) {
          e.printStackTrace();
          targetPose = _driveTrain.getPose();
        }
      }

    } else if(bestTag == 4){
      double targetX = AprilTagPoses.getPose(4).getX()-.5334+.04;
      double targetY = AprilTagPoses.getPose(4).getY();
      if(_offset == Offset.RIGHT) {
        targetY-=.762;
      }
      else {
        targetY+=.762;
      }
      Rotation2d targetRotation = new Rotation2d(0);
      targetPose = new Pose2d(targetX, targetY, targetRotation);
    } else{ //tag is 5
      double targetX = AprilTagPoses.getPose(5).getX()+.5334-.04;
      double targetY = AprilTagPoses.getPose(5).getY();
      if(_offset == Offset.RIGHT) {
        targetY+=.762;
      }
      else {
        targetY-=.762;
      }
      Rotation2d targetRotation = new Rotation2d(Math.PI);
      targetPose = new Pose2d(targetX, targetY, targetRotation);
    }


    
    
    // if(Math.abs(targetX-_driveTrain.getPose().getX())<.5) {
    //   _command = new FollowTrajectoryToPose(_driveTrain, new Pose2d((targetX+_driveTrain.getPose().getX())/2, targetY+1, targetRotation), .5);
    //   _command.initialize();
    // }

    if (side == null || grid == -1 || node == -1) {
      _command = new FollowTrajectoryToPose(_driveTrain, targetPose, .3);
    } else {
      _command = new FollowTrajectoryToPose(_driveTrain, targetPose, .3)
      .andThen(new TurnToAngle(_driveTrain, FieldPoses.getHighNodePose(side, grid, node), 1))
      .andThen(new DriveForDistance(_driveTrain, .5, .5));
    }
    _command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _command.isFinished() || _endEarly;
  }
}
