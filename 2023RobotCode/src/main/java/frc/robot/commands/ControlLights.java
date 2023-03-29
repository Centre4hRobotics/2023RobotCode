// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class ControlLights extends CommandBase {
  /** Creates a new ControlLights. */
  private Lights _lights;
  private Joystick _functionJoystick;
  public ControlLights(Lights lights, Joystick functionJoystick) {
    _lights = lights;
    _functionJoystick = functionJoystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_functionJoystick.getRawButton(4) ){//||_functionJoystick.getRawButton(5)) {
      _lights.setCone();
    }
    else if(_functionJoystick.getRawButton(6) ){//||_functionJoystick.getRawButton(7)) {
      _lights.setCube();
    }
    else {
      _lights.pulse2();
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