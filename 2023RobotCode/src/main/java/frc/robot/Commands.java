// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.FieldSide;
import frc.robot.commands.Autos;
import frc.robot.commands.StopDrive;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class Commands {
    private DriveTrain _driveTrain;
    public Commands(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
    }


    
    public Command selectCommand(String command) {
        // System.out.print("selecting command");
        // return Autos.balance(_driveTrain);

        try {
            switch(command) {
            case("Blue Center"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 1, 0));
                return Autos.scoreCenter(_driveTrain, FieldSide.LEFT, 0);
            case("Blue Right"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 0, 0));
                return Autos.bottomAutoThree(_driveTrain, FieldSide.LEFT, 0, 0);
            case("Blue Left"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 2, 2));
                return Autos.topAuto(_driveTrain, FieldSide.LEFT);
            case("Red Center"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.RIGHT, 1, 0));
                return Autos.scoreCenter(_driveTrain, FieldSide.RIGHT, 0);
            case("Red Right"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.RIGHT, 2, 2));
                return Autos.bottomAutoThree(_driveTrain, FieldSide.RIGHT, 2, 2);
            case("Red Left"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.RIGHT, 0, 0));
                return Autos.topAuto(_driveTrain, FieldSide.RIGHT);
            default:    
                return new StopDrive(_driveTrain);
            }
        } catch (Exception e) {
            NetworkTableInstance nt = NetworkTableInstance.getDefault();
            nt.getTable("_Errors").getEntry("Commands").setValue(e);
            return new StopDrive(_driveTrain);
        }
    }
}