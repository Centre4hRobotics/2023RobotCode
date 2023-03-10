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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.GroundControl;

/** Add your docs here. */
public class Commands {
    private DriveTrain _driveTrain;
    private GroundControl _groundControl;
    private Arm _arm;
    private Gripper _gripper;
    public Commands(DriveTrain driveTrain, GroundControl groundControl, Arm arm, Gripper gripper) {
        _driveTrain = driveTrain;
        _groundControl = groundControl;
        _arm = arm;
        _gripper = gripper;
    }


    
    public Command selectCommand(String command) {
        // System.out.print("selecting command");
        // return Autos.balance(_driveTrain);

        try {
            switch(command) {
            case("Blue Center"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 1, 0));
                return Autos.scoreCenter(_driveTrain, _arm, _gripper, FieldSide.LEFT, 0);
            case("Blue Right"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 0, 0));
                // return Autos.bottomAutoThree(_driveTrain, FieldSide.LEFT, 0, 0);
                return Autos.sideAuto(_driveTrain, _arm, _gripper, _groundControl, FieldSide.LEFT, 0, 0);
            case("Blue Left"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 2, 2));
                return Autos.sideAuto(_driveTrain, _arm, _gripper, _groundControl, FieldSide.LEFT, 2, 2);
            case("Red Center"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.RIGHT, 1, 0));
                return Autos.scoreCenter(_driveTrain, _arm, _gripper, FieldSide.RIGHT, 0);
            case("Red Right"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.RIGHT, 2, 2));
                //return Autos.bottomAutoThree(_driveTrain, FieldSide.RIGHT, 2, 2);
                return Autos.sideAuto(_driveTrain, _arm, _gripper, _groundControl, FieldSide.RIGHT, 2, 2);
            case("Red Left"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.RIGHT, 0, 0));
                return Autos.sideAuto(_driveTrain, _arm, _gripper, _groundControl, FieldSide.RIGHT, 0, 0);
            default:    
                return new StopDrive(_driveTrain);
            }
        } catch (Exception e) {
            NetworkTableInstance nt = NetworkTableInstance.getDefault();
            nt.getTable("@Errors").getEntry("Commands").setValue(e.getMessage().toString());
            return new StopDrive(_driveTrain);
        }
    }
}