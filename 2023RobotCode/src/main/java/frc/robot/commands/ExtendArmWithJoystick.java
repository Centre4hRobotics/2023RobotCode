// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ExtendArmWithJoystick extends CommandBase {
  /** Creates a new ExtendArmWithJoystick. */

  private Arm _arm;
  private Joystick _joystick;
  private boolean wasPressedLastCycle=false;

  public ExtendArmWithJoystick(Arm arm, Joystick joystick) {
    _arm = arm;
    _joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("ExtendArmWithJoystick").getEntry("joystick Y").setValue(_joystick.getY());
    nt.getTable("ExtendArmWithJoystick").getEntry("joystick X").setValue(_joystick.getX());
    nt.getTable("ExtendArmWithJoystick").getEntry("running").setValue(true);

    if(Math.abs(_joystick.getY()) > 0.5) {  //If the joystick is pressed
      _arm.extendVolts(-5.5*_joystick.getY());
      wasPressedLastCycle = true;
    }
    else if(wasPressedLastCycle) {
      _arm.extendVolts(0);
      _arm.setHeight(_arm.getHeight());
      wasPressedLastCycle = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
