// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Trajectories;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.FieldSide;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  public static SequentialCommandGroup scoreCenter(DriveTrain driveTrain, FieldSide side, int node) throws Exception {

    return new FollowTrajectory(driveTrain, Trajectories.generateScoreToCharge(side, 1, node, true))
      .andThen(new GetOnChargingStation(driveTrain, .5, -1))
      //.andThen(new DriveWithSpeed(driveTrain, -.5).withTimeout(.34))

      .andThen(new FollowTrajectoryToPose(driveTrain, FieldPoses.getOffChargingStationPose(side), .3))
      .andThen(new TurnToAngle(driveTrain, FieldPoses.getOnChargingStationPose(side), 5))
      .andThen(new GetOnChargingStation(driveTrain, .5, -1))

      .andThen(new Balance(driveTrain));
  }

  public static CommandBase balance(DriveTrain driveTrain) {
    return new Balance(driveTrain);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
