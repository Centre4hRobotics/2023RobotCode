// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Trajectories;
import frc.robot.Constants.Colors;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class GoToPosition extends SequentialCommandGroup {
  /** Creates a new GoToPosition. */
  public GoToPosition(DriveTrain drive, Pose2d position, Vision vision) {
    super(
      new UpdateOdometry(vision, drive, false),//Always update pose
      new ExampleCommand(drive, position)
      // new FollowTrajectory(drive, Trajectories.generateToPose(drive.getPose(), position))
    );
  }

  public GoToPosition(DriveTrain drive, Pose2d position) {
    super(
      new FollowTrajectory(drive, Trajectories.generateToPose(drive.getPose(), position, false))
    );
  }
}
