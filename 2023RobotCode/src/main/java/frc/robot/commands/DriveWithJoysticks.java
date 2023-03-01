// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveTrain;

public class DriveWithJoysticks extends CommandBase {
  private final DriveTrain _driveTrain;
  private final Joystick _left;
  private final Joystick _right;
  JoystickButton l4;

  /** Creates a new DriveWithJoystick. */
  public DriveWithJoysticks(DriveTrain driveTrain, Joystick left, Joystick right) {
    l4 = new JoystickButton(left, 4);
    _driveTrain = driveTrain;
    _left = left;
    _right = right;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = -1 * _left.getY();
    double right = -1 * _right.getY();

    // _driveTrain.tankDriveVolts(left * -5 * (_right.getThrottle()-1), right * -5 * (_right.getThrottle()-1));
    double throttle = (_right.getThrottle()-1)/-2; // gets from -1 to 1 to 1 to 0 ()
    throttle=.4+.6*throttle; // converts to 50% to 100% from 0% to 100%
    left *= 12*throttle;
    right *= 12*throttle;
    double diff = (left-right)/4;
    double avg = (left+right)/2;
    double leftVolts = avg;
    double rightVolts = avg;
    leftVolts += diff;
    rightVolts -= diff;

    //cut power by 1/4 if right trigger is pressed
    if(_right.getRawButton(1)) {
      leftVolts*=0.25;
      rightVolts*=0.25;
    }

    _driveTrain.tankDriveVolts(leftVolts, rightVolts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Stop the robot
    _driveTrain.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}