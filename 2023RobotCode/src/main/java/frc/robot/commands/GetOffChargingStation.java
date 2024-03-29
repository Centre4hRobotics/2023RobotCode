// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class GetOffChargingStation extends CommandBase {

  private DriveTrain _driveTrain;
  private double _speed, _direction;
  private final double targetAngle = 10;
  private final double targetLow = 3;//was 14.5
  private boolean passedBump = false;

  private NetworkTableInstance nt = NetworkTableInstance.getDefault();

  /** 
   * Drive partially on to the charging station. 
   * @param driveTrain
   * @param speed driving speed, 0 to 1
   * @param direction 1 or -1 (forwards or backwards)
   * */
  public GetOffChargingStation(DriveTrain driveTrain, double speed, double direction) {
    _driveTrain = driveTrain;
    _speed = speed;
    _direction = direction;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    nt.getTable("Get Off Charging Station").getEntry("initialized").setValue(true);
    passedBump = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    _driveTrain.arcadeDrive(_speed * _direction, 0);

    if (!passedBump && Math.abs(_driveTrain.getRoll()) > targetAngle) {
      passedBump = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return passedBump && Math.abs(_driveTrain.getRoll()) < targetLow;
  }
}
