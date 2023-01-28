// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

/** Add your docs here. */
public class Trajectories {

    //to create a new trajectory, add the trajectory here,
    //initilize it as a command in Commands.java's ,constructor
    //add the command to the switch statment in Commands.java, 
    //and add the trajectory's name to autoselector in RobotContainer.java
    //optionally, you can also make a new config for your path

    //Drive forward trajectory
    public static Trajectory reverse = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(7.716, 2.764, new Rotation2d(0.312, 0.805)),
        new Pose2d(7.469, 2.073, new Rotation2d(0.214, 0.608))
      ),
      getNewConfig(.5, .5).setReversed(true)
    );
    public static Trajectory reverse2 = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(7.633, 2.747, new Rotation2d(0.312, 0.805)),
        new Pose2d(6.795, 0.446, new Rotation2d(0.247, 0.625))
      ),
      getNewConfig(.5, .5).setReversed(true)
    );


    //Return the trajectory for a two-ball auto starting in bottom blue tarmac on the top left side
    public static Trajectory getTwoBallCenter () {
      ControlVectorList list = new TrajectoryGenerator.ControlVectorList();
      list.add(new Spline.ControlVector(
        new double[] {6.521, -.5, 0},
        new double[] {2.604, -0.223, 0}));
      list.add(new Spline.ControlVector(
        new double[] {5.087, -0.436, 0},
        new double[] {1.905, -0.322, 0}));
      
      return TrajectoryGenerator.generateTrajectory(list, getNewConfig(.4, .4));
    }
    public static Trajectory getTwoBallCenterPt2 () {
      ControlVectorList list = new TrajectoryGenerator.ControlVectorList();
      list.add(new Spline.ControlVector(
        new double[] {5.087, 1.183, 0},
        new double[] {1.905, -0.115, 0}));
        list.add(new Spline.ControlVector(
        new double[] {7.173, 1.183, 0},
        new double[] {1.662, -0.115, 0}));
      return TrajectoryGenerator.generateTrajectory(list, getNewConfig(.4, .4));
    }

    
    public Trajectory generateToPose(Pose2d startPosition, Pose2d endPosition) {
      return TrajectoryGenerator.generateTrajectory(
        List.of(startPosition, endPosition),
        getNewConfig(.4, .4)
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
            10);

        //Create trajectoryConfig object
        TrajectoryConfig config =
        new TrajectoryConfig(velocityCoefficient*Constants.kMaxSpeedMetersPerSecond,  //Run at 75%
            accelerationCoefficient*Constants.kMaxAccelerationMetersPerSecondSquared);  //Run at full acceleration
        config.setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);

        return config;
    }
}