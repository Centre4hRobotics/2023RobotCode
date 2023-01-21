// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.StopDrive;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class Commands {
    private Command testCommand;
    private DriveTrain _driveTrain;
    public Commands(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
    }

    public void initTrajectories() {
        testCommand = new SequentialCommandGroup(
        new FollowTrajectory(_driveTrain, Trajectories.reverse)
        );
    }

    
    public Command selectCommand(String command) {
        
        switch(command) {
        case("1-ball(LOW) Fender"):
            _driveTrain.resetOdometry(Trajectories.reverse.getInitialPose());
            return testCommand;
        default:
            return new StopDrive(_driveTrain);
        }
    }
}