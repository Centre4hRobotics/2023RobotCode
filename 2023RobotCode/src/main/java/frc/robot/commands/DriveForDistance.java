// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveForDistance extends CommandBase {

  private DriveTrain _driveTrain;
  private double _distance;
  private double _speed;

  private double initialLeft, initialRight;
  private boolean goneOver = false;
  private final double tolerance = .01;

  /** 
   * Drive for a set distance for a speed, based on encoder values 
   * *no steering
  */
  public DriveForDistance(DriveTrain driveTrain, double distance, double speed) {
    _driveTrain = driveTrain;
    _distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialLeft = _driveTrain.getLeftEncoder();
    initialRight = _driveTrain.getRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_distance - Math.abs(_driveTrain.getLeftEncoder() - initialLeft) > 0) {
      _driveTrain.arcadeDrive(_speed, 0);
    } else {
      goneOver = true;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return goneOver || (
      Math.abs(Math.abs(_driveTrain.getLeftEncoder() - initialLeft) - _distance) < tolerance &&
      Math.abs(Math.abs(_driveTrain.getRightEncoder() - initialRight) - _distance) < tolerance
    );
  }
}
