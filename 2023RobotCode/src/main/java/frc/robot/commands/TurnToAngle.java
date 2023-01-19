// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurnToAngleConstants;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  /** Creates a new TurnToAngle. */

  private ShuffleboardTab tab = Shuffleboard.getTab("Tune TurnToAngle PID");
  
  private GenericEntry P = tab.add("P", TurnToAngleConstants.kp)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
  private GenericEntry I = tab.add("I", TurnToAngleConstants.ki)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
  private GenericEntry IRangeTable = tab.add("IRange", TurnToAngleConstants.IRange)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();//Max might not be high enough
  private GenericEntry D = tab.add("D", TurnToAngleConstants.kd)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
  private GenericEntry baseEntry = tab.add("base", TurnToAngleConstants.base)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();


  double kp, ki, kd, IRange, base;
  PIDController _PidController;
  DriveTrain _driveTrain;
  double _targetAngle;
  double lastAngle = 0;

  public TurnToAngle(DriveTrain driveTrain, double targetAngle, double tolerance) {
    _driveTrain = driveTrain;
    _targetAngle = targetAngle;

    _PidController = new PIDController(kp, ki, kd);
    _PidController.setTolerance(tolerance);
    _PidController.setIntegratorRange(-1, 1);
    
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Grab PID info from shuffleboard and apply it
    setPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(_targetAngle-_driveTrain.getAngle())<IRange) {
      // changing the behavior of IRange to accumulate integral only while within IRange, instead of using it as a max for the accumulation.
      _PidController.setPID(kp, ki, kd);
    }
    else {
      _PidController.setPID(kp, 0, kd);
    }
    double pidValue = -_PidController.calculate(_driveTrain.getAngle(), _targetAngle);
    if(pidValue>0) { //adds a base motor power to overcome friction
      pidValue+=base;
    }
    else {
      pidValue-=base;
    }
    double maxVal = 1.0;
    if(pidValue>maxVal) { // caps pidValue to maxVal
      pidValue = maxVal;
    }
    else if(pidValue<-maxVal) {
      pidValue=-maxVal;
    }
    _driveTrain.arcadeDrive(0, pidValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _PidController.atSetpoint()&&Math.abs((lastAngle-_driveTrain.getAngle()))<TurnToAngleConstants.maxEndVelocity;
  }

  private void setPID() {
    kp = P.getDouble(TurnToAngleConstants.kp);
    ki = I.getDouble(TurnToAngleConstants.ki);
    IRange = this.IRangeTable.getDouble(TurnToAngleConstants.IRange);
    kd = D.getDouble(TurnToAngleConstants.kd);
    base = baseEntry.getDouble(TurnToAngleConstants.base);
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("PID").getEntry("P").setValue(kp);
    nt.getTable("PID").getEntry("I").setValue(ki);
    nt.getTable("PID").getEntry("D").setValue(kd);
    nt.getTable("PID").getEntry("IRange").setValue(IRange);
    nt.getTable("PID").getEntry("base power").setValue(base);

  }
}
