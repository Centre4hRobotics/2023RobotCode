// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Trajectories;
import frc.robot.subsystems.DriveTrain;

public class FollowTrajectoryToPose extends CommandBase {
  /** Creates a new FollowTrajectoryToPose. */

  DriveTrain _driveTrain;
  Pose2d _position;
  RamseteCommand _command;

  public FollowTrajectoryToPose(DriveTrain driveTrain, Pose2d position) {
    _driveTrain = driveTrain;
    _position = position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _command = new FollowTrajectory(_driveTrain, Trajectories.generateToPose(_driveTrain.getPose(), _position));
    _command.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Current pose: " + _driveTrain.getPose());
    System.out.println("Target pose: " + _position);
    // _command.execute();
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
