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
import frc.robot.subsystems.DriveTrain;

public class LockPosition extends CommandBase {

  private ShuffleboardTab tab = Shuffleboard.getTab("Tune LockPosition PID");
  
  private GenericEntry PEntry, IEntry, IRangeEntry, DEntry, baseEntry;
  private double kp, ki, kd, IRange, base;

  private DriveTrain _driveTrain;
  private double initialLeft, initialRight;

  private PIDController _PidController;

  /** Creates a new LockPosition. */
  public LockPosition(DriveTrain driveTrain) {
    _PidController = new PIDController(kp, ki, kd);
    _driveTrain = driveTrain;

    if (tab.getComponents().isEmpty()) {
      PEntry = tab.add("P", _driveTrain.getLockPositionkP())
        .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
      IEntry = tab.add("I", _driveTrain.getLockPositionkI())
        .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
      IRangeEntry = tab.add("IRange", _driveTrain.getLockPositionIRange())
        .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
      DEntry = tab.add("D", _driveTrain.getLockPositionkD())
        .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
      baseEntry = tab.add("base", _driveTrain.getLockPositionBase())
        .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
    } 
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrain);
  }

  private void setPID() {
    if (PEntry == null) {
      kp = _driveTrain.getLockPositionkP();
      ki = _driveTrain.getLockPositionkI();
      IRange = _driveTrain.getLockPositionIRange();
      kd = _driveTrain.getLockPositionkD();
      base = _driveTrain.getLockPositionBase();
    }
    else {
      kp = PEntry.getDouble(_driveTrain.getLockPositionkP());
      ki = IEntry.getDouble(_driveTrain.getLockPositionkI());
      IRange = IRangeEntry.getDouble(_driveTrain.getLockPositionIRange());
      kd = DEntry.getDouble(_driveTrain.getLockPositionkD());
      base = baseEntry.getDouble(_driveTrain.getLockPositionBase());
    }

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Lock Position PID").getEntry("P").setValue(kp);
    nt.getTable("Lock Position PID").getEntry("I").setValue(ki);
    nt.getTable("Lock Position PID").getEntry("D").setValue(kd);
    nt.getTable("Lock Position PID").getEntry("IRange").setValue(IRange);
    nt.getTable("Lock Position PID").getEntry("base power").setValue(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setPID();

    initialLeft = _driveTrain.getLeftEncoder();
    initialRight = _driveTrain.getRightEncoder();
  }

  private double getPIDValue(double current, double target) {
    if(Math.abs(current) < IRange) {
      // changing the behavior of IRange to accumulate integral only while within IRange, instead of using it as a max for the accumulation.
      _PidController.setPID(kp, ki, kd);
    }
    else {
      _PidController.setPID(kp, 0, kd);
    }
    double pidValue = _PidController.calculate(current, target);
    
    if(pidValue>0) { //adds a base motor power to overcome friction
      pidValue+=base;
    }
    else {
      pidValue-=base;
    }
    double maxVal = 3;
    if(pidValue>maxVal) { // caps pidValue to maxVal
      pidValue = maxVal;
    }
    else if(pidValue<-maxVal) {
      pidValue=-maxVal;
    }

    return pidValue;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double differenceLeft = _driveTrain.getLeftEncoder() - initialLeft;
    double differenceRight = _driveTrain.getRightEncoder() - initialRight;

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("Lock Position PID").getEntry("Left Encoder Difference")
      .setValue(differenceLeft);
    nt.getTable("Lock Position PID").getEntry("Right Encoder Difference")
      .setValue(differenceRight);

    nt.getTable("Lock Position PID").getEntry("Left PID Value")
      .setValue(getPIDValue(_driveTrain.getLeftEncoder(), initialLeft));
    nt.getTable("Lock Position PID").getEntry("Right PID Value")
      .setValue(getPIDValue(_driveTrain.getRightEncoder(), initialRight));

    _driveTrain.tankDriveVolts(
      getPIDValue(_driveTrain.getLeftEncoder(), initialLeft), 
      getPIDValue(_driveTrain.getRightEncoder(), initialRight)
    );
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
}
