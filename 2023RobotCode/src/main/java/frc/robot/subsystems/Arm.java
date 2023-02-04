// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private final CANSparkMax _leadMotor = new CANSparkMax(10, MotorType.kBrushless);
  // type is PneumaticsModuleType.CTREPCM or PneumaticsModuleType.REVPH
  private final DoubleSolenoid _doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  
  


  /** Creates a new Arm. */
  public Arm() {
    super();
    _leadMotor.getPIDController().setP(.02); // change only this one
    _leadMotor.getPIDController().setI(.002);
    _leadMotor.getPIDController().setD(0);
    _leadMotor.getPIDController().setIZone(10);
    _leadMotor.getPIDController().setFF(0);
    _leadMotor.getPIDController().setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Arm").getEntry("encoderValue").setValue(_leadMotor.getEncoder().getPosition());
  }

  public void raise() {
    _doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  public void lower() {
    _doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  //4.167 @ 0"
  //-6.357 @ 2.375"
  //-50.857 @ 13.375"
  //-152.591 @ 36.125"
  //-193.372 @ 45.6875"

  public void extendVolts(double volts) {
    _leadMotor.setVoltage(volts);
  }

  public void setHeightTop() {
    _leadMotor.getPIDController().setReference(-180, CANSparkMax.ControlType.kPosition);
  }

  public void setHeightMiddle() {
    _leadMotor.getPIDController().setReference(-90, CANSparkMax.ControlType.kPosition);
  }

  public void setHeightBottom() {
    _leadMotor.getPIDController().setReference(0, CANSparkMax.ControlType.kPosition);
  }

}
