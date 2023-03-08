// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;
import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.FieldSide;
import frc.robot.subsystems.DriveTrain;

/** Stores Trajectories. */
public class Trajectories {

    //to create a new trajectory, add the trajectory here,
    //initilize it as a command in Commands.java's ,constructor
    //add the command to the switch statment in Commands.java, 
    //and add the trajectory's name to autoselector in RobotContainer.java
    //optionally, you can also make a new config for your path

    
    
    public static Trajectory generateToPose(Pose2d startPosition, Pose2d endPosition, boolean reversed, double velocityCoefficient) {
      return TrajectoryGenerator.generateTrajectory(
        List.of(startPosition, endPosition),
        getNewConfig(velocityCoefficient, .9).setReversed(reversed)
      );
    }
    public static Trajectory generateToPose(List<Pose2d> points, boolean reversed, double velocityCoefficient) {
      return TrajectoryGenerator.generateTrajectory(
      points,
      getNewConfig(velocityCoefficient, .9).setReversed(reversed)
      );
    }

    public static Trajectory generateToPoseFromDrive(DriveTrain drive, Pose2d endPosition) {
      return TrajectoryGenerator.generateTrajectory(
        List.of(drive.getPose(), endPosition),
        getNewConfig(.1, .1)
      );
    }

    public static Trajectory generateScoreToCharge(FieldSide side, int grid, int node, boolean reversed) throws Exception {
      return generateToPose(
        FieldPoses.getScoringPose(side, grid, node), 
        FieldPoses.getOnChargingStationPose(side),
        reversed,
        .35//Max velocity
      );
    }

    public static Trajectory generateScoreToStage(FieldSide side, int grid, int node, int stagePosition, double velocityCoefficient, boolean useWaypoint) throws Exception {
      if(useWaypoint) {
        return generateToPose(List.of(
          FieldPoses.getScoringPose(side, grid, node), 
          FieldPoses.getAvoidChargingStationPose(side, grid==0, true), 
          FieldPoses.getStagingPose(side, stagePosition)),
          true, velocityCoefficient);
      }
      return generateToPose(
        FieldPoses.getScoringPose(side, grid, node),
        FieldPoses.getStagingPose(side, stagePosition),
        true,
        velocityCoefficient
      );
    }

    public static Trajectory generateScoreToStage(FieldSide side, int grid, int node, int stagePosition, double velocityCoefficient, double angle, boolean useWaypoint) throws Exception {
      if(useWaypoint) {
        return generateToPose(List.of(
          FieldPoses.getScoringPose(side, grid, node), 
          FieldPoses.getAvoidChargingStationPose(side, grid==0, true), 
          FieldPoses.getStagingPose(side, stagePosition, angle)), 
          true, velocityCoefficient);
      }
      return generateToPose(
        FieldPoses.getScoringPose(side, grid, node),
        FieldPoses.getStagingPose(side, stagePosition, angle),
        true,
        velocityCoefficient
      );
    }

    /**
     * Get list of poses EXCLUDING starting position
     */
    public static List<Pose2d> getScoreToStagePoses(FieldSide side, int grid, int node, int stagePosition, double velocityCoefficient, double angle, boolean useWaypoint) throws Exception {
      return List.of(
        FieldPoses.getAvoidChargingStationPose(side, grid==0, true), 
        FieldPoses.getStagingPose(side, stagePosition, angle));
    }

    public static Trajectory generateStageToScore(FieldSide side, int grid, int node, int stagePosition, double velocityCoefficient, boolean useWaypoint) throws Exception {
      if(useWaypoint) {
        return generateToPose(List.of(
          FieldPoses.getStagingPose(side, stagePosition),
          FieldPoses.getAvoidChargingStationPose(side, grid==0, true),
          FieldPoses.getScoringPose(side, grid, node)
        ), false, velocityCoefficient);
      }
      return generateToPose(
        FieldPoses.getStagingPose(side, stagePosition),
        FieldPoses.getScoringPose(side, grid, node),
        false,
        velocityCoefficient
      );
    }

    public static Trajectory generateScoreToSideStage(FieldSide side, int grid, int node, double velocityCoefficient) throws Exception {
      Rotation2d rotation;
      if(grid==0 && side==FieldSide.LEFT) {
        rotation = new Rotation2d((-Math.PI/2)*1.2);
      }
      else if(grid==2 && side==FieldSide.RIGHT) {
        rotation = new Rotation2d((-Math.PI/2)*1.2+Math.PI);
      }
      else if(grid==0 && side==FieldSide.RIGHT) {
        rotation = new Rotation2d((Math.PI/2)*1.2);
      }
      else {
        rotation = new Rotation2d((Math.PI/2)*1.2+Math.PI);
      }
      return generateToPose(List.of(
        FieldPoses.getScoringPose(side, grid, node),
        FieldPoses.getAvoidChargingStationPose(side, grid==0, true),
        FieldPoses.getAvoidGamePiecePose(side, grid),
        new Pose2d(FieldPoses.getTrueStagingPose(side, grid==0?1:2/*if grid==0 return 1 else 2*/).getTranslation(), rotation)
      ), true, velocityCoefficient);
    }
    
    public static TrajectoryConfig getNewConfig(double velocityCoefficient, double accelerationCoefficient) {
        
        //caps the velocity and acceleration coefficients at 1
        velocityCoefficient=Math.min(velocityCoefficient, 1);
        accelerationCoefficient=Math.min(accelerationCoefficient, 1);

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltsSecondsPerMeter,
                Constants.kaVoltsSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10); // 10

        //Create trajectoryConfig object
        TrajectoryConfig config =
        new TrajectoryConfig(velocityCoefficient*Constants.kMaxSpeedMetersPerSecond,  //Run at 75%
            accelerationCoefficient*Constants.kMaxAccelerationMetersPerSecondSquared);  //Run at full acceleration
        config.setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);

        return config;
    }
}