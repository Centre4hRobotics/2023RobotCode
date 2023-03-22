// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AprilTagPoses;
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
    _vision.updateOdomentry(_driveTrain);
    double targetX;
    double targetY;
    Rotation2d targetRotation;
    if(_driveTrain.getPose().getX()<8) {
      //tag 8
      targetX = AprilTagPoses.getPose(8).getX()+.5334;
      targetY = AprilTagPoses.getPose(8).getY();
      if(_offset == Offset.RIGHT) {
        targetY+=.762;
      }
      else {
        targetY-=.762;
      }
      targetRotation = new Rotation2d(Math.PI);
      if(Math.abs(targetX-_driveTrain.getPose().getX())<.5) {
        _command = new FollowTrajectoryToPose(_driveTrain, new Pose2d((targetX+_driveTrain.getPose().getX())/2, targetY+1, targetRotation), .5);
        _command.schedule();
      }
    }
    else {
      //tag 4
      targetX = AprilTagPoses.getPose(4).getX()-.5334;
      targetY = AprilTagPoses.getPose(4).getY();
      if(_offset == Offset.RIGHT) {
        targetY-=.762;
      }
      else {
        targetY+=.762;
      }
      targetRotation = new Rotation2d(0);
      if(Math.abs(targetX-_driveTrain.getPose().getX())<.5) {
        _command = new FollowTrajectoryToPose(_driveTrain, new Pose2d((targetX+_driveTrain.getPose().getX())/2, targetY-1, targetRotation), .5);
        _command.schedule();
      }
    }
    _command = new FollowTrajectoryToPose(_driveTrain, new Pose2d(targetX, targetY, targetRotation), .5);
    _command.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _command.isFinished();
  }
}
