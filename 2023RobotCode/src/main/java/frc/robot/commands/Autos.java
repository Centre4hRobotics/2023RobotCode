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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
    // new FollowTrajectory(driveTrain, Trajectories.generateScoreToCharge(side, 1, node, true))
    return 
      new GetOnChargingStation(driveTrain, .5, -1)
      // .andThen(new FollowTrajectoryToPose(driveTrain, FieldPoses.getOffChargingStationPose(side), .3))
      .andThen(new GetOffChargingStation(driveTrain, .5, -1))
      .andThen(new DriveWithSpeed(driveTrain, -.5).withTimeout(.25))
      .andThen(new TurnToAngle(driveTrain, FieldPoses.getOnChargingStationPose(side), 5))
      .andThen(new GetOnChargingStation(driveTrain, .5, -1))

      .andThen(new Balance(driveTrain));
  }

  public static CommandBase bottomAuto(DriveTrain driveTrain, FieldSide side) throws Exception {
    return new FollowTrajectory(driveTrain, Trajectories.generateScoreToStage(side, 0, 0, 0, .35))
      .andThen(new TurnToAngle(driveTrain, -180, 3)) // angle is relative to starting angle...
      .andThen(new WaitCommand(1))
      .andThen(new TurnToAngle(driveTrain, 0, 3))
      .andThen(new FollowTrajectory(driveTrain, Trajectories.generateStageToScore(side, 0, 0, 0, .35)));
  }

  public static CommandBase bottomAutoThree(DriveTrain driveTrain, FieldSide side) throws Exception {
    double velocityCoefficient = .5;
    return new FollowTrajectory(driveTrain, Trajectories.generateScoreToStage(side, 0, 0, 0, velocityCoefficient))
      .andThen(new TurnToAngle(driveTrain, -180, 3)) // angle is relative to starting angle...
      .andThen(new WaitCommand(1))
      .andThen(new TurnToAngle(driveTrain, 0, 3))
      .andThen(new FollowTrajectory(driveTrain, Trajectories.generateStageToScore(side, 0, 0, 0, velocityCoefficient)))
      .andThen(new WaitCommand(1))
      .andThen(new FollowTrajectory(driveTrain, 
        Trajectories.generateScoreToStage(side, 0, 0, 1, velocityCoefficient, Math.PI / 2)))
      .andThen(new TurnToAngle(driveTrain, -180, 3))
      .andThen(new WaitCommand(1))
      .andThen(new TurnToAngle(driveTrain, 0, 3))
      .andThen(new FollowTrajectory(driveTrain, Trajectories.generateStageToScore(side, 0, 0, 1, velocityCoefficient)));
      
  }

  public static CommandBase topAuto(DriveTrain driveTrain, FieldSide side) throws Exception {
    return new FollowTrajectory(driveTrain, Trajectories.generateScoreToStage(side, 2, 2, 3, .35))
      .andThen(new TurnToAngle(driveTrain, 0, 3));
  }

  public static CommandBase balance(DriveTrain driveTrain) {
    return new Balance(driveTrain);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
