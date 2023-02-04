// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ExtendArmWithJoystick extends CommandBase {
  /** Creates a new ExtendArmWithJoystick. */

  private Arm _arm;
  private Joystick _joystick;
  private double _voltage;

  public ExtendArmWithJoystick(Arm arm, Joystick joystick, double voltage) {
    _arm = arm;
    _joystick = joystick;
    _voltage = voltage;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _arm.extendVolts(_joystick.getX() * _voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _arm.extendVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}