// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GroundControl;

public class IntakeWithSwitch extends CommandBase {
  /** Creates a new IntakeWithSwitch. */
  private GroundControl _groundControl;
  private Joystick _functionJoystick;
  private double _speed;
  public IntakeWithSwitch(GroundControl groundControl, Joystick functionJoystick, double speed) {
    _groundControl = groundControl;
    _functionJoystick = functionJoystick;
    _speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_groundControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!_functionJoystick.getRawButton(3)) {
      _groundControl.setSpeed(_speed);
    }
    else {
      _groundControl.setSpeed(0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _groundControl.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
