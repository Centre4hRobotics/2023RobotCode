// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Trajectories;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.FieldSide;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {


  public static SequentialCommandGroup scoreCenter(DriveTrain driveTrain, FieldSide side, int node) throws Exception {
    // new FollowTrajectory(driveTrain, Trajectories.generateScoreToCharge(side, 1, node, true))
    return 
      new GetOnChargingStation(driveTrain, .5, -1)
      // .andThen(new FollowTrajectoryToPose(driveTrain, FieldPoses.getOffChargingStationPose(side), .3))
      .andThen(new GetOffChargingStation(driveTrain, .5, -1))
      .andThen(new DriveWithSpeed(driveTrain, -.5).withTimeout(.25))
      .andThen(new TurnToAngle(driveTrain, 180, 5))
      .andThen(new GetOnChargingStation(driveTrain, .5, -1))
      .andThen(new Balance(driveTrain));
  }

  public static CommandBase bottomAuto(DriveTrain driveTrain, FieldSide side) throws Exception {
    return new FollowTrajectory(driveTrain, Trajectories.generateScoreToStage(side, 0, 0, 0, .35, false))
      .andThen(new TurnToAngle(driveTrain, -180, 3)) // angle is relative to starting angle...
      .andThen(new WaitCommand(1))
      .andThen(new TurnToAngle(driveTrain, 0, 3))
      .andThen(new FollowTrajectory(driveTrain, Trajectories.generateStageToScore(side, 0, 0, 0, .35, false)));
  }

  public static CommandBase bottomAutoThree(DriveTrain driveTrain, FieldSide side, int grid, int node) throws Exception {
    double velocityCoefficient = .5;
    double angle=.67617;
    if(side==FieldSide.RIGHT) {
      angle-=Math.PI;
    }
    angle+=Math.PI;
    return new FollowTrajectory(driveTrain, Trajectories.generateScoreToStage(side, grid, node, grid==0?1:2, velocityCoefficient, angle, true))
      .andThen(new TurnToAngle(driveTrain, FieldPoses.getTrueStagingPose(side, grid==0?1:2), 3).withTimeout(.5))
      .andThen(new WaitCommand(.5))
      .andThen(new TurnToAngle(driveTrain, FieldPoses.getAvoidChargingStationPose(side, grid==0, true), 3).withTimeout(.5))
      .andThen(new FollowTrajectory(driveTrain, Trajectories.generateStageToScore(side, grid, node, grid==0?1:2, velocityCoefficient, true)))
      .andThen(new WaitCommand(.5))
      .andThen(new FollowTrajectory(driveTrain, Trajectories.generateScoreToSideStage(side, grid, node, velocityCoefficient)))
      .andThen(new TurnToAngle(driveTrain, 90, 3))
      .andThen(new DriveWithSpeed(driveTrain, velocityCoefficient).withTimeout(.1/velocityCoefficient))
      .andThen(new WaitCommand(.5))
      .andThen(new TurnToAngle(driveTrain, 20, 3).withTimeout(.5))
      .andThen(new FollowTrajectory(driveTrain, Trajectories.generateStageToScore(side, grid, 1, grid==0?0:3, velocityCoefficient, true)));
      
  }

  public static CommandBase topAuto(DriveTrain driveTrain, FieldSide side) throws Exception {
    return new FollowTrajectory(driveTrain, Trajectories.generateScoreToStage(side, 2, 2, 3, .35, false))
      .andThen(new TurnToAngle(driveTrain, 0, 3));
  }

  public static CommandBase balance(DriveTrain driveTrain) {
    return new Balance(driveTrain);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
