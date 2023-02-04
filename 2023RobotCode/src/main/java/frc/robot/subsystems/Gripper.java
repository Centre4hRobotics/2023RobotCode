// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

  private final Solenoid _Solenoid = new Solenoid(null, 2);

  /** Creates a new Gripper. */
  public Gripper() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void open() {
    _Solenoid.set(true);
  }

  public void close() {
    _Solenoid.set(false);
  }
}
