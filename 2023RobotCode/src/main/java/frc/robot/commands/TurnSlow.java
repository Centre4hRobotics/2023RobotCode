// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnSlow extends CommandBase {
  /** Creates a new TurnSlow. */
  private DriveTrain _driveTrain;
  private boolean _clockwise;
  private int _frame=0;
  public TurnSlow(DriveTrain driveTrain, boolean clockwise) {
    _driveTrain = driveTrain;
    _clockwise = clockwise;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _frame=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double posVol = 0;
    double negVol = -4;
    if(_frame!=0 && _frame<4) {
      if(_clockwise) {
        _driveTrain.tankDriveVolts(negVol, posVol);
      }
      else {
        _driveTrain.tankDriveVolts(posVol, negVol);
      }
    }
    else {
      if(_clockwise) {
        _driveTrain.tankDriveVolts(negVol/2, posVol/2);
      }
      else {
        _driveTrain.tankDriveVolts(posVol/2, negVol/2);
      }
    }
    _frame++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    // return _frame>=5;
  }
}
