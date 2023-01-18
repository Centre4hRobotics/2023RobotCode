// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  /** Creates a new TurnToAngle. */

  private ShuffleboardTab tab = Shuffleboard.getTab("Tune PID");
  private GenericEntry setPID = tab.add("Set PID Values", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
  // private NetworkTableEntry setPID = tab.add("Set PID Values", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
  private Boolean setPIDState = false;
  
  private GenericEntry P = tab.add("P", 0)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
  private GenericEntry I = tab.add("I", 0)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
  private GenericEntry IRangeTable = tab.add("IRange", 0)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0, "max", 1)).getEntry();//Max might not be high enough
  private GenericEntry D = tab.add("D", 0)
    .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();

  // private GenericEntry TargetAngle = tab.add("Target Angle", 0)
  //   .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", -1080, "max", 1080)).getEntry();//I don't know if the range is correct


  double kp, ki, kd, IRange;
  PIDController _PidController;
  DriveTrain _driveTrain;
  double _targetAngle;

  public TurnToAngle(DriveTrain driveTrain, double targetAngle, double tolerance) {
    _driveTrain = driveTrain;
    _targetAngle = targetAngle;

    kp = _driveTrain.getkP();
    ki = _driveTrain.getkI();
    IRange = _driveTrain.getIRange();
    kd = _driveTrain.getkD();

    _PidController = new PIDController(kp, ki, kd);
    _PidController.setTolerance(tolerance);
    _PidController.setIntegratorRange(-IRange, IRange);
    
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {// new NetworkButton(intakeEnable).whenPressed(this::setPID);
    setPIDState = setPID.getBoolean(false);

    //Grab PID info from shuffleboard and apply it
    setPID();
    Timer.delay(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setPID();
    _PidController.setPID(kp, ki, kd);
    _PidController.setIntegratorRange(-IRange, IRange);
    double pidValue = -_PidController.calculate(_driveTrain.getAngle(), _targetAngle);
    double maxVal = .8;
    if(pidValue>maxVal) {
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
    return _PidController.atSetpoint();
  }

  private void setPID() {
    kp = P.getDouble(0);
    ki = I.getDouble(0);
    IRange = this.IRangeTable.getDouble(0);
    kd = D.getDouble(0);
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("PID").getEntry("P").setValue(kp);
    nt.getTable("PID").getEntry("I").setValue(ki);
    nt.getTable("PID").getEntry("D").setValue(kd);
    nt.getTable("PID").getEntry("IRange").setValue(IRange);

  }
}
