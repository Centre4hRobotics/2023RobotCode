// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Trajectories;
import frc.robot.Constants.FieldSide;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {

  public static CommandBase reverse(DriveTrain _driveTrain) {
    return new SequentialCommandGroup(
      new FollowTrajectory(_driveTrain, Trajectories.reverse)
    );
  }

  public static CommandBase test(DriveTrain driveTrain) {
    return new FollowTrajectory(driveTrain, Trajectories.test);
  }

  public static CommandBase scoreToCharge(DriveTrain driveTrain, FieldSide side, int grid, int node, double angle) throws Exception {
    return new SequentialCommandGroup(
      new FollowTrajectory(driveTrain, Trajectories.generateScoreToCharge(side, grid, node, angle, true))
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
