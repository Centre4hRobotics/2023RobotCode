// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.FieldSide;
import frc.robot.commands.Autos;
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
        try {
            switch(command) {
            case("1-ball(LOW) Fender"):
                _driveTrain.resetOdometry(Trajectories.reverse.getInitialPose());
                return testCommand;
            case("test"):
                _driveTrain.resetOdometry(Trajectories.test.getInitialPose());
                return Autos.test(_driveTrain);
            case("Blue Grid 0, Node 0"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 0, 0));
                return Autos.scoreToCharge(_driveTrain, FieldSide.LEFT, 0, 0);
            case("Blue Grid 1, Node 0"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 1, 0));
                return Autos.scoreToCharge(_driveTrain, FieldSide.LEFT, 1, 0);
            case("Blue Grid 2, Node 0"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 2, 0));
                return Autos.scoreToCharge(_driveTrain, FieldSide.LEFT, 2, 0);
            case("balance"):
                return Autos.balance(_driveTrain);
            default:    
                return new StopDrive(_driveTrain);
            }
        } catch (Exception e) {
            return new StopDrive(_driveTrain);
        }
    }
}