// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Trajectories;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.FieldSide;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BoxingGloves;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.GroundControl;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {  

  public static SequentialCommandGroup scoreCenter(DriveTrain driveTrain, Arm arm, Gripper gripper, FieldSide side, int node) throws Exception {
    // new FollowTrajectory(driveTrain, Trajectories.generateScoreToCharge(side, 1, node, true))
    return 
      score(arm, gripper, ArmConstants.highPosition)
      .andThen(new DriveForDistance(driveTrain, 2, -.7))
      .andThen(new DriveForDistance(driveTrain, 2, -.5))
      .andThen(new WaitCommand(1))
      .andThen(new DriveForDistance(driveTrain, 1.8, .7))
      .andThen(new Balance(driveTrain));
      // .andThen(new GetOnChargingStation(driveTrain, .5, -1))
      // .andThen(new GetOffChargingStation(driveTrain, .5, -1))
      // .andThen(new DriveWithSpeed(driveTrain, -.5).withTimeout(1))
      // .andThen(new TurnToAngle(driveTrain, 180, 5))
      // .andThen(new GetOnChargingStation(driveTrain, .5, -1))
      // .andThen(new BasicBalance(driveTrain, .3, -1))
      // .andThen(new LockPosition(driveTrain));


      // .andThen(new Balance(driveTrain));
      // return 
      // score(arm, gripper, ArmConstants.highPosition)
      // //.andThen(new GetOnChargingStation(driveTrain, .6, -1).withTimeout(1.85))//Was .5
      // .andThen(new DriveForDistance(driveTrain, 2, -.7))
      // .andThen(new Balance(driveTrain));
  }
  public static SequentialCommandGroup scoreCenterExpiremental(DriveTrain driveTrain, Arm arm, Gripper gripper, GroundControl groundControl, FieldSide side, int node) throws Exception {
    double d = .7;
    return 
      scoreWithoutRetract(arm, gripper, ArmConstants.highPosition)
        .andThen(((new SetArmHeight(arm, ArmConstants.retracted))
        .andThen(new LowerArm(arm)))
        .alongWith(new DriveForDistance(driveTrain, .25, -.5)
        .andThen(new DriveForDistance(driveTrain, 1.75, -.7))))
      .andThen(new DriveForDistance(driveTrain, 1.8, -.5))
      .andThen(new WaitCommand(.1))
      .andThen(new TurnToAngle(driveTrain, 180, 5).withTimeout(1.75))
      .andThen(new LowerGroundControl(groundControl))
      .andThen(new WaitCommand(.25))
      .andThen(new ParallelDeadlineGroup(new DriveForDistance(driveTrain, d, .9), new Intake(groundControl, .8)))
      .andThen(new RaiseGroundControl(groundControl))
      .andThen(new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitCommand(.25),
          new TurnToAngle(driveTrain, 0, 5).withTimeout(1.5),
          new DriveForDistance(driveTrain, d+2.2, .7)),
        new Intake(groundControl, .4)))
      .andThen(new Balance(driveTrain)
      .alongWith(new SequentialCommandGroup(
        new LowerGroundControl(groundControl),
        new WaitCommand(.27),
        new CloseGroundControl(groundControl),
        new Intake(groundControl, -1))
      ))
      ;
  }

  public static CommandBase sideAuto(DriveTrain driveTrain, Arm arm, Gripper gripper, GroundControl groundControl, FieldSide side, int grid, int node) throws Exception {
    double velocityCoefficient = .99;
    double angle=0;
    if(side==FieldSide.RIGHT) {
      angle-=Math.PI;
    }
    angle+=Math.PI;
    return 
      score(arm, gripper, ArmConstants.highPosition)
      .andThen(new FollowTrajectory(driveTrain, Trajectories.generateScoreToStage(side, grid, node, grid==0?0:3, velocityCoefficient, angle, true)))
      .andThen(new TurnToAngle(driveTrain, FieldPoses.getTrueStagingPose(side, grid==0?0:3), 3).withTimeout(1.5))
      .andThen(groundGrabWithMoveForward(driveTrain, groundControl, GamePiece.CONE))
      .andThen(new TurnToAngle(driveTrain, FieldPoses.getAvoidChargingStationPose(side, grid==0, true), 5).withTimeout(1.5))
      .andThen(new FollowTrajectory(driveTrain, Trajectories.generateStageToScore(side, grid, 1, grid==0?0:3, velocityCoefficient, true)))
      // .andThen(new FollowTrajectoryToPose(driveTrain, List.of(
      //   FieldPoses.getAvoidChargingStationPose(side, grid==0, true),
      //   FieldPoses.getScoringPose(side, grid, node)
      // ), false, velocityCoefficient))    
      .andThen(new LowerGroundControl(groundControl))
      .andThen(new WaitCommand(.2))
      .andThen(new Intake(groundControl, -.9).withTimeout(0.08))
      .andThen(new Intake(groundControl, -.285));
  }

  public static CommandBase sideJeremiahAuto(DriveTrain driveTrain, Arm arm, Gripper gripper, GroundControl groundControl, BoxingGloves boxingGloves, FieldSide side, int grid, int node) throws Exception {
    double velocityCoefficient = .99;
    double angle=0;
    if(side==FieldSide.RIGHT) {
      angle-=Math.PI;
    }
    angle+=Math.PI;
    return 
      // score high
      score(arm, gripper, ArmConstants.highPosition)
      // move to pos + gloves move
      .andThen(new ParallelDeadlineGroup(
        new FollowTrajectory(driveTrain, Trajectories.generateScoreToStage(side, grid, node, grid==0?0:3, velocityCoefficient, angle, true)),
        new RaiseBoxingGloves(boxingGloves)
          .andThen(new WaitCommand(.25))
          .andThen(new LowerBoxingGloves(boxingGloves))
      ))
      
      // turn grab and go back
      .andThen(new TurnToAngle(driveTrain, 180, 3).withTimeout(1.5))
      .andThen(groundGrabWithMoveForward(driveTrain, groundControl, GamePiece.CUBE))
      .andThen(new ParallelDeadlineGroup(
        new TurnToAngle(driveTrain, FieldPoses.getAvoidChargingStationPose(side, grid==0, true), 5).withTimeout(1.5)
        .andThen(new FollowTrajectory(driveTrain, Trajectories.generateStageToScore(side, grid, 1, grid==0?0:3, velocityCoefficient, true)))
        , new Intake(groundControl, .4)))  
      // shooting sequence
      .andThen(new SequentialCommandGroup(
        new LowerGroundControl(groundControl),
        new WaitCommand(.2),
        new CloseGroundControl(groundControl),
        new Intake(groundControl, -1)))
      // drive backwards at verry end
      .andThen(new WaitCommand(.75))
      .andThen(new DriveWithSpeed(driveTrain, -.4));}

  public static CommandBase sideAutoTest(DriveTrain driveTrain, Arm arm, Gripper gripper, GroundControl groundControl, FieldSide side, int grid, int node, Vision vision) throws Exception {
    double velocityCoefficient = .4;
    double angle=0;
    if(side==FieldSide.RIGHT) {
      angle-=Math.PI;
    }
    angle+=Math.PI;
    return 
      score(arm, gripper, ArmConstants.highPosition)
      .andThen(new FollowTrajectory(driveTrain, Trajectories.generateScoreToStage(side, grid, node, grid==0?0:3, velocityCoefficient, angle, true)))
      .andThen(new TurnToAngle(driveTrain, FieldPoses.getTrueStagingPose(side, grid==0?0:3), 3).withTimeout(1.5))
      .andThen(groundGrabWithMoveForward(driveTrain, groundControl, GamePiece.CUBE))
      // .andThen(new ParallelDeadlineGroup(
      //   new TurnToAngle(driveTrain, side==FieldSide.LEFT?180:0, 3).withTimeout(2)
      //     .andThen(new FollowTrajectory(driveTrain, Trajectories.generateStageToVision(side, grid, grid==0?0:3, velocityCoefficient)))
      //     .andThen(new LowerGroundControl(groundControl))
      //     .andThen(new WaitCommand(.5))
      //     .andThen(new UpdateOdometry(vision, driveTrain, true))
      //     .andThen(new FollowTrajectoryToPose(driveTrain, FieldPoses.getScoringPose(side, grid, 1), velocityCoefficient)),
      //   new Intake(groundControl, .25)))
      .andThen(new TurnToAngle(driveTrain, side==FieldSide.LEFT?180:0, 3).withTimeout(1.5))
      .andThen(new FollowTrajectory(driveTrain, Trajectories.generateStageToVision(side, grid, grid==0?0:3, velocityCoefficient)))
      .andThen(new LowerGroundControl(groundControl))
      .andThen(new WaitCommand(.5))
      .andThen(new UpdateOdometry(vision, driveTrain, true))
      .andThen(new FollowTrajectoryToPose(driveTrain, FieldPoses.getScoringPose(side, grid, 1), velocityCoefficient))
      .andThen(new Intake(groundControl, -.8).withTimeout(.5));
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
      .andThen(groundGrabWithMoveForward(driveTrain, groundControl, GamePiece.CONE))
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
      new Log("Auto", "before set arm height")
      .andThen(new SetArmHeight(arm, height)
        .alongWith(new WaitCommand(.15).andThen(new LowerArm(arm))))
      .andThen(new Log("Auto", "after set arm height"))
      .andThen(new OpenGripper(gripper))
      .andThen(new Log("Auto", "after open gripper"))
      .andThen(new WaitCommand(.15))//was .25
      .andThen(new SetArmHeight(arm, ArmConstants.retracted))
      .andThen(new RaiseArm(arm));
  }
  public static SequentialCommandGroup scoreWithoutRetract(Arm arm, Gripper gripper, double height){
    return
      new Log("Auto", "before set arm height")
      .andThen(new SetArmHeight(arm, height)
        .alongWith(new WaitCommand(.15).andThen(new LowerArm(arm))))
      .andThen(new Log("Auto", "after set arm height"))
      .andThen(new OpenGripper(gripper))
      .andThen(new Log("Auto", "after open gripper"))
      .andThen(new WaitCommand(.15));
  }

  public static CommandBase scoreWithMoveBack(DriveTrain driveTrain, Arm arm, Gripper gripper, double height) {
    return
      (new SetArmHeight(arm, height)
        .alongWith(new WaitCommand(.15).andThen(new LowerArm(arm))))
      .andThen(new OpenGripper(gripper))
      .andThen(new WaitCommand(.25))
      .andThen(new ParallelDeadlineGroup(
        new SetArmHeight(arm, ArmConstants.retracted),
        new DriveWithSpeed(driveTrain, -.4)
      ))
      .andThen(new RaiseArm(arm));
  }

  public static SequentialCommandGroup groundGrabWithMoveForward(DriveTrain driveTrain, GroundControl groundControl, GamePiece piece) {
    if (piece == GamePiece.CONE) {
      return 
      new LowerGroundControl(groundControl)
      .andThen(new WaitCommand(.5))
      .andThen(new ParallelDeadlineGroup(
        new DriveForDistance(driveTrain, .5, .5),
        new Intake(groundControl, .3)))
      //.andThen(new DriveWithSpeed(driveTrain, .5).withTimeout(.85))//was .3, .4, .65
      // .andThen(new FollowTrajectoryToPose(driveTrain, -.2, 0, .3))
      .andThen(new CloseGroundControl(groundControl))
      .andThen(new WaitCommand(.25))
      .andThen(new RaiseGroundControl(groundControl))
      .andThen(new WaitCommand(.5));
    } else {
      return 
      new LowerGroundControl(groundControl)
      .andThen(new WaitCommand(.5))
      .andThen(new ParallelDeadlineGroup(
        new DriveForDistance(driveTrain, .6, .55),
        new Intake(groundControl, .4)))
      .andThen(new RaiseGroundControl(groundControl))
      .andThen(new WaitCommand(.5));
    }
  }

  public static SequentialCommandGroup groundGrabWithSmallMoveForward(DriveTrain driveTrain, GroundControl groundControl, GamePiece piece) {
    if (piece == GamePiece.CONE) {
      return 
      new LowerGroundControl(groundControl)
      .andThen(new WaitCommand(.5))
      .andThen(new ParallelDeadlineGroup(
        new DriveForDistance(driveTrain, .25, .5),
        new Intake(groundControl, .3)))
      //.andThen(new DriveWithSpeed(driveTrain, .5).withTimeout(.85))//was .3, .4, .65
      // .andThen(new FollowTrajectoryToPose(driveTrain, -.2, 0, .3))
      .andThen(new CloseGroundControl(groundControl))
      .andThen(new WaitCommand(.25))
      .andThen(new RaiseGroundControl(groundControl))
      .andThen(new WaitCommand(.5));
    } else {
      return 
      new LowerGroundControl(groundControl)
      .andThen(new WaitCommand(.5))
      .andThen(new ParallelDeadlineGroup(
        new DriveForDistance(driveTrain, .25, .55),
        new Intake(groundControl, .4)))
      .andThen(new RaiseGroundControl(groundControl))
      .andThen(new WaitCommand(.5));
    }
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}