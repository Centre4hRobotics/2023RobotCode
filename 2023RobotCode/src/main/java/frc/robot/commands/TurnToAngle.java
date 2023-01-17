// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  /** Creates a new TurnToAngle. */
  double kp, ki, kd, IRange;
  PIDController _PidController;
  DriveTrain _driveTrain;
  double _targetAngle;

  public TurnToAngle(DriveTrain driveTrain, double targetAngle, double tolerance) {
    _driveTrain = driveTrain;
    _targetAngle = targetAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrain);
    kp = _driveTrain.getkP();
    ki = _driveTrain.getkI();
    IRange = _driveTrain.getIRange();
    kd = _driveTrain.getkD();

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("PID").getEntry("P").setValue(kp);
    nt.getTable("PID").getEntry("I").setValue(ki);
    nt.getTable("PID").getEntry("D").setValue(kd);
    nt.getTable("PID").getEntry("IRange").setValue(IRange);

    _PidController = new PIDController(kp, ki, kd);
    _PidController.setTolerance(tolerance);
    _PidController.setIntegratorRange(-IRange, IRange);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidValue = _PidController.calculate(_driveTrain.getAngle(), _targetAngle);
    if(pidValue>1) {
      pidValue = 1;
    }
    else if(pidValue<-1) {
      pidValue=-1;
    }
    _driveTrain.arcadeDrive(0, pidValue); //+ or - pidValue
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
}
