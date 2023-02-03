// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundControl extends SubsystemBase {
  /** Creates a new GroundControl. */

  private final Solenoid _leftUpDownSolenoid = new Solenoid(null, 0);
  private final Solenoid _rightUpDownSolenoid = new Solenoid(null, 0);
  private final Solenoid _leftOpenCloseSolenoid = new Solenoid(null, 0);
  private final Solenoid _rightOpenCloseSolenoid = new Solenoid(null, 0);

  private final CANSparkMax _leadMotor = new CANSparkMax(0, null);
  private final CANSparkMax _followMotor = new CANSparkMax(0, null);

  boolean _isUp, _isOpen;

  public GroundControl() {
    _followMotor.follow(_leadMotor, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _leftUpDownSolenoid.set(_isUp);
    _rightUpDownSolenoid.set(_isUp);
    _leftOpenCloseSolenoid.set(_isOpen);
    _rightOpenCloseSolenoid.set(_isOpen);
  }

  public void setVoltage(double volts) {
    _leadMotor.setVoltage(volts);
  }

  public boolean getIsUp() {
    return _isUp;
  }

  public void setIsUp(boolean isUp) {
    _isUp = isUp;
  }

  public boolean getIsOpen() {
    return _isOpen;
  }

  public void setIsOpen(boolean isOpen) {
    _isOpen = isOpen;
  }
}
