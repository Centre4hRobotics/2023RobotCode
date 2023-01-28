// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

  Solenoid _leftSolenoid = new Solenoid(null, 0);
  Solenoid _rightSolenoid = new Solenoid(null, 0);

  /** Creates a new Gripper. */
  public Gripper() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void open(boolean isRun) {
    _leftSolenoid.set(isRun);
    _rightSolenoid.set(isRun);
  }
}
