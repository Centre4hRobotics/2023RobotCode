// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  /** Creates a new TurnToAngle. */

  private static ShuffleboardTab tab = Shuffleboard.getTab("Tune TurnToAngle PID");
  private static GenericEntry P;
  private static GenericEntry I;
  private static GenericEntry IRangeTable;
  private static GenericEntry D;
  private static GenericEntry baseEntry;
  
  private Pose2d _target = null;


  double kp, ki, kd, IRange, base;
  PIDController _PidController;
  DriveTrain _driveTrain;
  double _targetAngle;
  double lastAngle = 0;
  double lastVel = 0;

  public TurnToAngle(DriveTrain driveTrain, double targetAngle, double tolerance) {
    _driveTrain = driveTrain;
    _targetAngle = targetAngle;

    initializePID(tolerance);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrain);
  }

  public TurnToAngle(DriveTrain driveTrain, Pose2d target, double tolerance) {
    _driveTrain = driveTrain;
    _target = target;

    initializePID(tolerance);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrain);
  }
 
  private void initializePID(double tolerance) {
    if (tab.getComponents().isEmpty()) {
      P = tab.add("P", _driveTrain.getTurnToAnglekP())
        .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
      I = tab.add("I", _driveTrain.getTurnToAnglekI())
        .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
      IRangeTable = tab.add("IRange", _driveTrain.getTurnToAngleIRange())
        .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();//Max might not be high enough
      D = tab.add("D", _driveTrain.getTurnToAnglekD())
        .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
      baseEntry = tab.add("base", _driveTrain.getTurnToAngleBase())
        .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0)).getEntry();
    } 

    _PidController = new PIDController(kp, ki, kd);
    _PidController.setTolerance(tolerance);
    _PidController.setIntegratorRange(-1, 1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Grab PID info from shuffleboard and apply it
    if (_target != null) {
      Pose2d currentPose = _driveTrain.getPose();
      _targetAngle = Math.atan2(_target.getY() - currentPose.getY(), _target.getX() - currentPose.getX()) * 180 / Math.PI;

      NetworkTableInstance nt = NetworkTableInstance.getDefault();
      nt.getTable("TurnToAngle PID").getEntry("Target Angle").setValue(_targetAngle);
    }

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
    double maxAccel = _driveTrain.getTurnToAngleMaxAcceleration();
    if(pidValue>lastVel) {
      if(pidValue-lastVel>maxAccel) {
        pidValue = lastVel+maxAccel;
      }
    }
    else if(pidValue-lastVel<-maxAccel) {
      pidValue = lastVel-maxAccel;
    }
    lastVel = pidValue;
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
    return _PidController.atSetpoint()&&Math.abs((lastAngle-_driveTrain.getAngle()))<_driveTrain.getTurnToAngleMaxEndVelocity();
  }

  private void setPID() {
    kp = P.getDouble(_driveTrain.getTurnToAnglekP());
    ki = I.getDouble(_driveTrain.getTurnToAnglekI());
    IRange = this.IRangeTable.getDouble(_driveTrain.getTurnToAngleIRange());
    kd = D.getDouble(_driveTrain.getTurnToAnglekD());
    base = baseEntry.getDouble(_driveTrain.getTurnToAngleBase());
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("TurnToAngle PID").getEntry("P").setValue(kp);
    nt.getTable("TurnToAngle PID").getEntry("I").setValue(ki);
    nt.getTable("TurnToAngle PID").getEntry("D").setValue(kd);
    nt.getTable("TurnToAngle PID").getEntry("IRange").setValue(IRange);
    nt.getTable("TurnToAngle PID").getEntry("base power").setValue(base);

  }
}
