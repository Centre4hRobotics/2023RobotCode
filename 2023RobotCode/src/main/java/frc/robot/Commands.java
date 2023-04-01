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
import frc.robot.subsystems.BoxingGloves;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.GroundControl;
import frc.robot.subsystems.Vision;

/** Add your docs here. */
public class Commands {
    private DriveTrain _driveTrain;
    private GroundControl _groundControl;
    private Arm _arm;
    private Gripper _gripper;
    private Vision _vision;
    private BoxingGloves _boxingGloves;

    private Command blueLeft;
    private Command blueCenter;
    private Command blueRight;
    private Command redLeft;
    private Command redCenter;
    private Command redRight;
    

    public Commands(DriveTrain driveTrain, GroundControl groundControl, Arm arm, Gripper gripper, Vision vision, BoxingGloves boxingGloves) {
        _driveTrain = driveTrain;
        _groundControl = groundControl;
        _arm = arm;
        _gripper = gripper;
        _vision = vision;
        _boxingGloves = boxingGloves;
        try {
            blueLeft = Autos.sideAuto(_driveTrain, _arm, _gripper, _groundControl, FieldSide.LEFT, 2, 2);
            blueCenter = Autos.scoreCenter(_driveTrain, _arm, _gripper, FieldSide.LEFT, 0);
            blueRight = Autos.sideAuto(_driveTrain, _arm, _gripper, _groundControl, FieldSide.LEFT, 0, 0);
            redLeft = Autos.sideAuto(_driveTrain, _arm, _gripper, _groundControl, FieldSide.RIGHT, 0, 0);
            redCenter = Autos.scoreCenterExpiremental(_driveTrain, _arm, _gripper, _groundControl, FieldSide.RIGHT, 0);
            redRight = Autos.sideJeremiahAuto(_driveTrain, _arm, _gripper, _groundControl, _boxingGloves, FieldSide.RIGHT, 2, 2);
        }
        catch (Exception e) {
            NetworkTableInstance nt = NetworkTableInstance.getDefault();
            nt.getTable("@Errors").getEntry("Commands").setValue(e.getMessage().toString());
        }
    }


    
    public Command selectCommand(String command) {
        // System.out.print("selecting command");
        // return Autos.balance(_driveTrain);

        try {
            switch(command) {
            case("Blue Center"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 1, 0));
                return blueCenter;
            case("Blue Right"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 0, 0));
                // return Autos.bottomAutoThree(_driveTrain, FieldSide.LEFT, 0, 0);
                return blueRight;
            case("Blue Left"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 2, 2));
                return blueLeft;
            case("Red Center"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.RIGHT, 1, 0));
                return redCenter;
            case("Red Right"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.RIGHT, 2, 2));
                //return Autos.bottomAutoThree(_driveTrain, FieldSide.RIGHT, 2, 2);
                //return Autos.sideAuto(_driveTrain, _arm, _gripper, _groundControl, FieldSide.RIGHT, 2, 2);
                return redRight;
            case("Red Left"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.RIGHT, 0, 0));
                return redLeft;
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