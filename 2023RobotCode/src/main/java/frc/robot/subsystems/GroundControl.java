// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundControl extends SubsystemBase {
  /** Creates a new GroundControl. */

  private final Solenoid _UpDownSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  private final Solenoid _OpenCloseSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

  private final TalonSRX _leadMotor = new TalonSRX(11);//right
  private final TalonSRX _followMotor = new TalonSRX(23);//left

  boolean _isUp, _isOpen;

  public GroundControl() {
    _followMotor.follow(_leadMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    _leadMotor.set(TalonSRXControlMode.PercentOutput, speed);
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
