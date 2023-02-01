// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmHeight extends CommandBase {
  /** Creates a new SetArmHeight. */
  private final Arm _arm;
  private final int _height;
  public SetArmHeight(Arm arm, int height) {
    _arm = arm;
    _height = height;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(_height) {
      case 1:
        _arm.setHeightBottom();
        break;
      case 2:
        _arm.setHeightMiddle();
        break;
      case 3:
        _arm.setHeightTop();
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
