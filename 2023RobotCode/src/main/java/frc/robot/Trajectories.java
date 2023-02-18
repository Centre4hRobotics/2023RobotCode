// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
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

    public static Trajectory generateScoreToStage(FieldSide side, int grid, int node, int stagePosition, double velocityCoefficient) throws Exception {
      return generateToPose(
        FieldPoses.getScoringPose(side, grid, node),
        FieldPoses.getStagingPose(side, stagePosition),
        true,
        velocityCoefficient
      );
    }

    public static Trajectory generateScoreToStage(FieldSide side, int grid, int node, int stagePosition, double velocityCoefficient, double angle) throws Exception {
      return generateToPose(
        FieldPoses.getScoringPose(side, grid, node),
        FieldPoses.getStagingPose(side, stagePosition, angle),
        true,
        velocityCoefficient
      );
    }

    public static Trajectory generateStageToScore(FieldSide side, int grid, int node, int stagePosition, double velocityCoefficient) throws Exception {
      return generateToPose(
        FieldPoses.getStagingPose(side, stagePosition),
        FieldPoses.getScoringPose(side, grid, node),
        false,
        velocityCoefficient
      );
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
            4); // 10

        //Create trajectoryConfig object
        TrajectoryConfig config =
        new TrajectoryConfig(velocityCoefficient*Constants.kMaxSpeedMetersPerSecond,  //Run at 75%
            accelerationCoefficient*Constants.kMaxAccelerationMetersPerSecondSquared);  //Run at full acceleration
        config.setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);

        return config;
    }
}