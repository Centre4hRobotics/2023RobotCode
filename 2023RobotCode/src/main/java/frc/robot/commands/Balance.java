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
import frc.robot.Constants.BalanceConstants;
import frc.robot.subsystems.DriveTrain;

public class Balance extends CommandBase {
  /** Creates a new Balance. */

  private ShuffleboardTab tab = Shuffleboard.getTab("Tune Balance PID");
  
  private GenericEntry P = tab.add("P", BalanceConstants.kp)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
  private GenericEntry I = tab.add("I", BalanceConstants.ki)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
  private GenericEntry IRangeTable = tab.add("IRange", BalanceConstants.IRange)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();//Max might not be high enough
  private GenericEntry D = tab.add("D", BalanceConstants.kd)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
  private GenericEntry baseEntry = tab.add("base", BalanceConstants.base)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();


  DriveTrain _driveTrain;

  double kp, ki, kd, IRange, base;
  PIDController _PidController;

  

  public Balance(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    _PidController = new PIDController(kp, ki, kd);
    _driveTrain = driveTrain;
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(_driveTrain.getRobotPitch())<IRange) {
      // changing the behavior of IRange to accumulate integral only while within IRange, instead of using it as a max for the accumulation.
      _PidController.setPID(kp, ki, kd);
    }
    else {
      _PidController.setPID(kp, 0, kd);
    }
    double pidValue = _PidController.calculate(_driveTrain.getRobotPitch(), 0);
    if(pidValue>0) { //adds a base motor power to overcome friction
      pidValue+=base;
    }
    else {
      pidValue-=base;
    }
    double maxVal = .3;
    if(pidValue>maxVal) { // caps pidValue to maxVal
      pidValue = maxVal;
    }
    else if(pidValue<-maxVal) {
      pidValue=-maxVal;
    }
    _driveTrain.arcadeDrive(pidValue, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void setPID() {
    kp = P.getDouble(BalanceConstants.kp);
    ki = I.getDouble(BalanceConstants.ki);
    IRange = this.IRangeTable.getDouble(BalanceConstants.IRange);
    kd = D.getDouble(BalanceConstants.kd);
    base = baseEntry.getDouble(BalanceConstants.base);
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Balance PID").getEntry("P").setValue(kp);
    nt.getTable("Balance PID").getEntry("I").setValue(ki);
    nt.getTable("Balance PID").getEntry("D").setValue(kd);
    nt.getTable("Balance PID").getEntry("IRange").setValue(IRange);
    nt.getTable("Balance PID").getEntry("base power").setValue(base);

  }

}
