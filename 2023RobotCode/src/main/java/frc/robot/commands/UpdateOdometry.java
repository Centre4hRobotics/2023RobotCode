// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class UpdateOdometry extends CommandBase {

  Vision _vision;
  DriveTrain _driveTrain;

  /** Creates a new UpdateOdometry. */
  public UpdateOdometry(Vision vision, DriveTrain drive) {
    _vision = vision;
    _driveTrain = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _vision.updateOdomentry(_driveTrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}