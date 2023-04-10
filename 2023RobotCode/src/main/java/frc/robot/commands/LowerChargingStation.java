// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;

public class LowerChargingStation extends CommandBase {
  DriveTrain _driveTrain;
  Command initalCommand;
  Command redoCommand;
  Command _command;

  /** Creates a new LowerChargingStation. */
  public LowerChargingStation(DriveTrain driveTrain) {
    _driveTrain = driveTrain;

    initalCommand = new SequentialCommandGroup(
      new DriveForDistance(driveTrain, 2, -.7).withTimeout(1.5),
      new WaitCommand(.5),
      new DriveForDistance(driveTrain, .55, -.2).withTimeout(1.5)
    );
    redoCommand = new SequentialCommandGroup(
      new DriveForDistance(driveTrain, 0.5, .6).withTimeout(1.5),
      new WaitCommand(.95),
      new DriveForDistance(driveTrain, 2.5, -.7).withTimeout(1.5),
      new WaitCommand(.5),
      new DriveForDistance(driveTrain, .55, -.2).withTimeout(1.5)
    );
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _command = initalCommand;

    _command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _command.end(interrupted);
    _driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(_command.isFinished()){

      if(Math.abs(_driveTrain.getRoll()) > 8){
        return true;
      } else {
        _command = redoCommand;
        _command.initialize();
      }
      
    }
    
    return false;
  }
}
