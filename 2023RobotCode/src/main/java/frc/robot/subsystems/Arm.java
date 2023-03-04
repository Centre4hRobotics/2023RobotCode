// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private final CANSparkMax _leadMotor = new CANSparkMax(16, MotorType.kBrushless);
  // type is PneumaticsModuleType.CTREPCM or PneumaticsModuleType.REVPH
  private final DoubleSolenoid _doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);
  
  private GroundControl _groundControl;
  private double _height = 0;


  /** Creates a new Arm. */
  public Arm() {
    super();
    _leadMotor.getPIDController().setP(.022); // change only this one
    _leadMotor.getPIDController().setI(0.00005);
    _leadMotor.getPIDController().setD(0);
    _leadMotor.getPIDController().setIZone(15);
    _leadMotor.getPIDController().setFF(0);
    _leadMotor.getPIDController().setOutputRange(-1, 1);

    _leadMotor.setSmartCurrentLimit(20);

    _leadMotor.getEncoder().setPosition(0);
  }

  public void get(GroundControl groundControl) {
    _groundControl = groundControl;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Arm").getEntry("encoderValue").setValue(_leadMotor.getEncoder().getPosition());
    // _leadMotor.getPIDController().setOutputRange(-5, 5);
  }

  public void raise() {
    if (!isExtended()) {
      _doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }
  public void lower() {
    if(!isExtended() && (_groundControl.isOpen() || _groundControl.isDown())) {
      _doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  }
  public boolean isRaised() {
    return _doubleSolenoid.get()==DoubleSolenoid.Value.kReverse;
  }
  public boolean isExtended() {
    return _leadMotor.getEncoder().getPosition()*ArmConstants.encoderTicksToMeters > ArmConstants.middlePosition / 2;
  }

  /**
   * @return The extension of the arm as a demical, 0.0 when fully retracted and 1.0 when fully extended
   */
  public double getExtension() {
    double current = _leadMotor.getEncoder().getPosition() * ArmConstants.encoderTicksToMeters;
    double val = (current - ArmConstants.retracted) / (ArmConstants.highPosition - ArmConstants.retracted);
    return Math.min(Math.max(val, 0), 1);
  }

  //4.167 @ 0"
  //-6.357 @ 2.375"
  //-50.857 @ 13.375"
  //-152.591 @ 36.125"
  //-193.372 @ 45.6875"
  // -0.23052 inches per tick
  // -0.005855208 meters per tick

  // 187.706 high
  // 114.810 mid
  // 26 low
  // 48.5 pickup
  // 

  public void extendVolts(double volts) {
    _leadMotor.setVoltage(volts);
  }

  public void setHeight(double position) {
    _height = position;
    position/=ArmConstants.encoderTicksToMeters;
    _leadMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }
  public double getHeight() {
    return _leadMotor.getEncoder().getPosition()*ArmConstants.encoderTicksToMeters;
  }

  public void resetEncoder() {
    _leadMotor.getEncoder().setPosition(0);
  }

  public boolean isOnTarget() {
    return Math.abs(getHeight() - _height) < .04; // meters
  }
}
