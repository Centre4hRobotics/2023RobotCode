// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Trajectories;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {

  public static CommandBase reverse(DriveTrain _driveTrain) {
    return new SequentialCommandGroup(
      new FollowTrajectory(_driveTrain, Trajectories.reverse)
    );
  }

  public static CommandBase test(DriveTrain driveTrain) {
    return new FollowTrajectory(driveTrain, Trajectories.test);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
