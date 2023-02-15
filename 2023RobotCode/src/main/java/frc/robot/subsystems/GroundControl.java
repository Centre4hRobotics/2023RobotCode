// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundControl extends SubsystemBase {
  /** Creates a new GroundControl. */

  private final Solenoid _UpDownSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  private final Solenoid _OpenCloseSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

  private final TalonSRX _leadMotor = new TalonSRX(11);//right
  private final TalonSRX _followMotor = new TalonSRX(123);//left
  private final Arm _arm;

  public GroundControl(Arm arm) {
    _followMotor.setInverted(true);
    _followMotor.follow(_leadMotor);
    _arm = arm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    _leadMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public boolean isDown() {
    return _UpDownSolenoid.get();
  }

  public boolean isOpen() {
    return !_OpenCloseSolenoid.get();
  }
  public void open() {
    _OpenCloseSolenoid.set(false);
  }

  public void close() {
    // if(_arm.isRaised() || isDown()) {
      _OpenCloseSolenoid.set(true);
    // }
  }

  public void raise() {
    // if(_arm.isRaised() || isOpen()) {
      _UpDownSolenoid.set(false);
    // }
  }

  public void lower() {
    _UpDownSolenoid.set(true);
  }

}
