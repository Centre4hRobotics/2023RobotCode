// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Trajectories;
import frc.robot.subsystems.DriveTrain;

public class FollowTrajectoryToPose extends CommandBase {
  /** Creates a new FollowTrajectoryToPose. */

  DriveTrain _driveTrain;
  private Pose2d _position;
  private ArrayList<Pose2d> _positions;

  double _offsetX, _offsetY, _maxVelocityCoefficient;
  private RamseteCommand _command;
  
  private DoubleLogEntry odometryX;
  private DoubleLogEntry odometryY;
  private DoubleLogEntry visionX;
  private DoubleLogEntry visionY;

  private boolean _reversed;

  public FollowTrajectoryToPose(DriveTrain driveTrain, Pose2d position, double maxVelocityCoef) {
    _driveTrain = driveTrain;
    _position = position;
    _maxVelocityCoefficient = maxVelocityCoef;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  /**
   * Drive to pose from current DriveTrain pose
   * @param driveTrain
   * @param positions points AFTER current DriveTrain pose
   * @param maxVelocityCoef
   */
  public FollowTrajectoryToPose(DriveTrain driveTrain, List<Pose2d> positions, boolean reversed, double maxVelocityCoef) {
    _driveTrain = driveTrain;
    _positions = new ArrayList<>();
    for (Pose2d p : positions) _positions.add(p);
    _maxVelocityCoefficient = maxVelocityCoef;
    _reversed = reversed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  public FollowTrajectoryToPose(DriveTrain driveTrain, double offsetX, double offsetY, double maxVelocityCoef) {
    _driveTrain = driveTrain;
    _offsetX = offsetX;
    _offsetY = offsetY;
    _maxVelocityCoefficient = maxVelocityCoef;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    // offset constructor
    if (_position == null && _positions == null) {
      _position = _driveTrain.getPose().transformBy(
        new Transform2d(new Translation2d(_offsetX, _offsetY), new Rotation2d(_driveTrain.getAngle()))
      );
    }

    // pose / offset constructor
    Trajectory bestTrajectory;
    if (_positions == null) {
      //Calculate distance from target pose to a little in front/behind robot
      double frontHypot = Math.hypot(
        _position.getX()-(_driveTrain.getPose().getX()+_driveTrain.getPose().getRotation().getCos()),
        _position.getY()-(_driveTrain.getPose().getY()+_driveTrain.getPose().getRotation().getSin())
      );
      double backHypot = Math.hypot(
        _position.getX()-(_driveTrain.getPose().getX()-_driveTrain.getPose().getRotation().getCos()),
        _position.getY()-(_driveTrain.getPose().getY()-_driveTrain.getPose().getRotation().getSin())
      );

      //Use these distances to determine if the trajectory should go forward or backward
      _reversed = backHypot < frontHypot;

      bestTrajectory = Trajectories.generateToPose(_driveTrain.getPose(), _position, _reversed, _maxVelocityCoefficient);
    } else {
      // current position constructor
      _positions.add(0, _driveTrain.getPose());
      bestTrajectory = Trajectories.generateToPose(_positions, _reversed, _maxVelocityCoefficient);
    }

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("@debug").getEntry("FollowTrajectory").setValue("Running " + bestTrajectory.getInitialPose().toString());

    _command = new FollowTrajectory(_driveTrain, bestTrajectory);
    _command.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _command.execute();
    visionX.append(_driveTrain.getPose().getX());
    visionY.append(_driveTrain.getPose().getY());
    odometryX.append(_driveTrain.getPose().getX());
    odometryY.append(_driveTrain.getPose().getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("@debug").getEntry("FollowTrajectory").setValue("Not Running");
    _command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _command.isFinished();
  }
}