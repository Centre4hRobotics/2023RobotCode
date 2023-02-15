// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  private DigitalOutput _out1 = new DigitalOutput(0);
  private DigitalOutput _out2 = new DigitalOutput(2);
  public Lights() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setOut(boolean value1, boolean value2) {
    _out1.set(value1);
    _out2.set(value2);
  }


  public void setOff() {
    _out1.set(false);
    _out2.set(false);
  }
  public void setCube() {
    _out1.set(false);
    _out2.set(true);
  }
  public void setCone() {
    _out1.set(true);
    _out2.set(false);
  }
  public void setFloorCube() {
    _out1.set(true);
    _out2.set(true);
  }
}
