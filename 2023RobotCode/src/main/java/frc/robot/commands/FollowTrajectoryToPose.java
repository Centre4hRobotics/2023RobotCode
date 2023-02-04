// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Trajectories;
import frc.robot.subsystems.DriveTrain;

public class FollowTrajectoryToPose extends CommandBase {
  /** Creates a new FollowTrajectoryToPose. */

  DriveTrain _driveTrain;
  Pose2d _position;
  double _offsetX, _offsetY;
  private RamseteCommand _command;
  
  private DoubleLogEntry odometryX;
  private DoubleLogEntry odometryY;
  private DoubleLogEntry visionX;
  private DoubleLogEntry visionY;

  public FollowTrajectoryToPose(DriveTrain driveTrain, Pose2d position) {
    _driveTrain = driveTrain;
    _position = position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  public FollowTrajectoryToPose(DriveTrain driveTrain, double offsetX, double offsetY) {
    _driveTrain = driveTrain;
    _offsetX = offsetX;
    _offsetY = offsetY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  

    if (_position == null) {
      // untested
      _position = _driveTrain.getPose().transformBy(
        new Transform2d(new Translation2d(_offsetX, _offsetY), new Rotation2d(_driveTrain.getAngle()))
      );
    }
    _command = new FollowTrajectory(_driveTrain, Trajectories.generateToPose(_driveTrain.getPose(), _position, false));
    _command.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // _command.execute();
    visionX.append(_driveTrain.getPose().getX());
    visionY.append(_driveTrain.getPose().getY());
    odometryX.append(_driveTrain.getPose().getX());
    odometryY.append(_driveTrain.getPose().getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _command.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _command.isFinished();
  }
}
