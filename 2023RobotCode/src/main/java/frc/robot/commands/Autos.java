// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Trajectories;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.FieldSide;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.GroundControl;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {


  public static SequentialCommandGroup scoreCenter(DriveTrain driveTrain, Arm arm, Gripper gripper, FieldSide side, int node) throws Exception {
    // new FollowTrajectory(driveTrain, Trajectories.generateScoreToCharge(side, 1, node, true))
    return 
      score(arm, gripper, ArmConstants.highPosition)
      .andThen(new GetOnChargingStation(driveTrain, .5, -1))
      // .andThen(new FollowTrajectoryToPose(driveTrain, FieldPoses.getOffChargingStationPose(side), .3))
      .andThen(new GetOffChargingStation(driveTrain, .5, -1))
      .andThen(new DriveWithSpeed(driveTrain, -.5).withTimeout(1))
      .andThen(new TurnToAngle(driveTrain, 180, 5))
      .andThen(new GetOnChargingStation(driveTrain, .5, -1))
      .andThen(new BasicBalance(driveTrain, .3, -1))
      .andThen(new LockPosition(driveTrain));
      // .andThen(new Balance(driveTrain));
  }

  public static CommandBase bottomAuto(DriveTrain driveTrain, Arm arm, Gripper gripper, GroundControl groundControl, FieldSide side, int grid, int node) throws Exception {
    double velocityCoefficient = .5;
    double angle=0;
    if(side==FieldSide.RIGHT) {
      angle-=Math.PI;
    }
    angle+=Math.PI;
    return score(arm, gripper, ArmConstants.highPosition)
      .andThen(new FollowTrajectory(driveTrain, Trajectories.generateScoreToStage(side, grid, node, grid==0?0:3, velocityCoefficient, angle, true)))
      .andThen(new TurnToAngle(driveTrain, FieldPoses.getTrueStagingPose(side, grid==0?0:3), 3).withTimeout(2))
      .andThen(new LowerGroundControl(groundControl))
      .andThen(new WaitCommand(.5))
      .andThen(new DriveWithSpeed(driveTrain, .35).withTimeout(.9))
      .andThen(new CloseGroundControl(groundControl))
      .andThen(new WaitCommand(.5))
      .andThen(new RaiseGroundControl(groundControl))
      .andThen(new WaitCommand(.5))
      .andThen(new TurnToAngle(driveTrain, FieldPoses.getAvoidChargingStationPose(side, grid==0, true), 3).withTimeout(.6))
      .andThen(new FollowTrajectory(driveTrain, Trajectories.generateStageToScore(side, grid, node, grid==0?0:3, velocityCoefficient, true)))
      .andThen(new LowerGroundControl(groundControl))
      .andThen(new WaitCommand(.5))
      .andThen(new OpenGroundControl(groundControl));
    
  }

  public static CommandBase bottomAutoThree(DriveTrain driveTrain, GroundControl groundControl, FieldSide side, int grid, int node) throws Exception {
    double velocityCoefficient = .5;
    double angle=.67617;
    if(side==FieldSide.RIGHT) {
      angle-=Math.PI;
    }
    angle+=Math.PI;
    return new FollowTrajectory(driveTrain, Trajectories.generateScoreToStage(side, grid, node, grid==0?1:2, velocityCoefficient, angle, true))
      .andThen(new TurnToAngle(driveTrain, FieldPoses.getTrueStagingPose(side, grid==0?1:2), 3).withTimeout(1))
      .andThen(new LowerGroundControl(groundControl))
      .andThen(new WaitCommand(.5))
      .andThen(new DriveWithSpeed(driveTrain, .35).withTimeout(.9))
      .andThen(new CloseGroundControl(groundControl))
      .andThen(new WaitCommand(.5))
      .andThen(new RaiseGroundControl(groundControl))
      .andThen(new TurnToAngle(driveTrain, FieldPoses.getAvoidChargingStationPose(side, grid==0, true), 3).withTimeout(.6))
      .andThen(new FollowTrajectory(driveTrain, Trajectories.generateStageToScore(side, grid, node, grid==0?1:2, velocityCoefficient, true)))
      .andThen(new LowerGroundControl(groundControl))
      .andThen(new WaitCommand(.5))
      .andThen(new OpenGroundControl(groundControl))
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

  /**
   * Command to lower and extend arm, open gripper, retract and then raise the arm
   * @param arm
   * @param level 1-3 with 1 for ground and 3 for highest
   * @return command to do all this
   */
  public static SequentialCommandGroup score(Arm arm, Gripper gripper, double height){
    return
      new LowerArm(arm)
      .andThen(new SetArmHeight(arm, height))
      .andThen(new OpenGripper(gripper))
      .andThen(new WaitCommand(.25))
      .andThen(new SetArmHeight(arm, ArmConstants.retracted))
      .andThen(new RaiseArm(arm));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
