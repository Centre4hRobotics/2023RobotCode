// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Trajectories;
import frc.robot.subsystems.DriveTrain;

public class FollowTrajectory extends RamseteCommand {
  /** Creates a new FollowTrajectory. */
  public FollowTrajectory(DriveTrain drive, Trajectory trajectory) {
    super(trajectory,
    drive::getPose,
    new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltsSecondsPerMeter, Constants.kaVoltsSecondsSquaredPerMeter),
    Constants.kDriveKinematics,
    drive::getWheelSpeeds,
    new PIDController(Constants.kpDriveVel, 0, 0),
    new PIDController(Constants.kpDriveVel, 0, 0),
    drive::tankDriveVolts,  
    drive);
    
    // NetworkTableInstance nt = NetworkTableInstance.getDefault();
    // nt.getTable("Follow Trajectory").getEntry("Initial Pose").setValue(trajectory.getInitialPose().toString());
    // nt.getTable("Follow Trajectory").getEntry("All Poses").setValue(trajectory.toString());
  }
}