// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private final CANSparkMax _leadMotor = new CANSparkMax(5, MotorType.kBrushless);
  // type is PneumaticsModuleType.CTREPCM or PneumaticsModuleType.REVPH
  private final Solenoid _leftSolenoid = new Solenoid(null, 0);
  private final Solenoid _rightSolenoid = new Solenoid(null, 0);

  /** Creates a new Arm. */
  public Arm() {
    super();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void lift(boolean isRun) {
    _leftSolenoid.set(isRun);
    _rightSolenoid.set(isRun);
  }

  public void extendVolts(double volts) {
    _leadMotor.setVoltage(volts);
  }

  
}
