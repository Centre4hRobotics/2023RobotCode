// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BoxingGloves extends SubsystemBase {
  /** Creates a new BoxingGloves. */
  //switch 5
  private final Solenoid _boxingGloves = new Solenoid(PneumaticsModuleType.CTREPCM, 5);
  public BoxingGloves() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void raise() {
    _boxingGloves.set(true);
  }

  public void lower() {
    _boxingGloves.set(false);
  }
}
