// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private final CANSparkMax _leadMotor = new CANSparkMax(5, MotorType.kBrushless);
  // type is PneumaticsModuleType.CTREPCM or PneumaticsModuleType.REVPH
  private final Solenoid _leftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  private final Solenoid _rightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);
  

  private boolean _isUp = false;

  /** Creates a new Arm. */
  public Arm() {
    super();
    _leadMotor.getPIDController().setP(0); // change only this one
    _leadMotor.getPIDController().setI(0);
    _leadMotor.getPIDController().setD(0);
    _leadMotor.getPIDController().setIZone(0);
    _leadMotor.getPIDController().setFF(0);
    _leadMotor.getPIDController().setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _leftSolenoid.set(_isUp);
    _rightSolenoid.set(_isUp);
    
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Arm").getEntry("encoderValue").setValue(_leadMotor.getEncoder().getPosition());
  }

  public void setIsUp(boolean isUp) {
    _isUp = isUp;
  }

  public boolean getIsUp() {
    return _isUp;
  }

  public void extendVolts(double volts) {
    _leadMotor.setVoltage(volts);
  }

  public void setHeightTop() {
    _leadMotor.getPIDController().setReference(0, CANSparkMax.ControlType.kPosition);
  }

  public void setHeightMiddle() {
    _leadMotor.getPIDController().setReference(0, CANSparkMax.ControlType.kPosition);
  }

  public void setHeightBottom() {
    _leadMotor.getPIDController().setReference(0, CANSparkMax.ControlType.kPosition);
  }

}
