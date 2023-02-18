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
            /*case("Blue Grid 0, Node 0"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 0, 0));
                return Autos.scoreToCharge(_driveTrain, FieldSide.LEFT, 0, 0);*/
            case("Blue Grid 1, Node 0"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 1, 0));
                return Autos.scoreCenter(_driveTrain, FieldSide.LEFT, 0);
           /*case("Blue Grid 2, Node 0"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 2, 0));
                return Autos.scoreToCharge(_driveTrain, FieldSide.LEFT, 2, 0);*/
            case("Blue Bottom"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 0, 0));
                return Autos.bottomAutoThree(_driveTrain, FieldSide.LEFT);
            case("Blue Top"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 2, 2));
                return Autos.topAuto(_driveTrain, FieldSide.LEFT);
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