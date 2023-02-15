// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ExtendArmWithButtons extends CommandBase {
  /** Creates a new ExtendArmWithButtons. */
  private final Arm _arm;
  private final Joystick _functionJoystick;
  public ExtendArmWithButtons(Arm arm, Joystick functionJoystick) {
    _arm = arm;
    _functionJoystick = functionJoystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_functionJoystick.getRawButton(5)) {
      // _arm.setHeight(_arm.getHeight()+.02);
      _arm.extendVolts(5);
    }
    else if(_functionJoystick.getRawButton(6)) {
      // _arm.setHeight(_arm.getHeight()-.02);
      _arm.extendVolts(-5);
    }
    else {
      _arm.extendVolts(0);
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
