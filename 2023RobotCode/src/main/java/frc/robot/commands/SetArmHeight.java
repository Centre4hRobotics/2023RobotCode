// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmHeight extends CommandBase {
  /** Creates a new SetArmHeight. */
  private final Arm _arm;
  private final double _height;
  public SetArmHeight(Arm arm, double height) {
    _arm = arm;
    _height = height;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _arm.setHeight(_height);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("@debug").getEntry("arm height").setValue("ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _arm.isOnTarget();
  }
}
