// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundControl extends SubsystemBase {
  /** Creates a new GroundControl. */

  private final Solenoid _UpDownSolenoid = new Solenoid(null, 3);
  private final Solenoid _OpenCloseSolenoid = new Solenoid(null, 4);

  private final CANSparkMax _leadMotor = new CANSparkMax(0, null);
  private final CANSparkMax _followMotor = new CANSparkMax(0, null);

  boolean _isUp, _isOpen;

  public GroundControl() {
    _followMotor.follow(_leadMotor, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVoltage(double volts) {
    _leadMotor.setVoltage(volts);
  }

  public void open() {
    _OpenCloseSolenoid.set(false);
  }

  public void close() {
    _OpenCloseSolenoid.set(true);
  }

  public void raise() {
    _UpDownSolenoid.set(false);
  }

  public void lower() {
    _UpDownSolenoid.set(true);
  }

}
