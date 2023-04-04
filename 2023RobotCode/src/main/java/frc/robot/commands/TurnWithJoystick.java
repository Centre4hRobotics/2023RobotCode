// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnWithJoystick extends CommandBase {
  DriveTrain _driveTrain;
  Joystick _joystick;

  /** Creates a new TurnWithJoystick. */
  public TurnWithJoystick(DriveTrain driveTrain, Joystick joystick) {
    _driveTrain = driveTrain;
    _joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double steer = _joystick.getTwist()*0.3;

    _driveTrain.arcadeDrive(0, steer);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
