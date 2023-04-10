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

    private Command left, center, right, leftMid, centerDouble, rightMid;
    private Command leftPunching, centerPunching, rightPunching, leftMidPunching, centerDoublePunching, rightMidPunching;
    

    public Commands(DriveTrain driveTrain, GroundControl groundControl, Arm arm, Gripper gripper, Vision vision, BoxingGloves boxingGloves) {
        _driveTrain = driveTrain;
        _groundControl = groundControl;
        _arm = arm;
        _gripper = gripper;
        _vision = vision;
        _boxingGloves = boxingGloves;
        try {
            left = Autos.sideAuto(_driveTrain, _arm, _gripper, _groundControl, FieldSide.LEFT, 2, 2, _boxingGloves, false);
            // center = Autos.scoreCenterWithRedo(_driveTrain, _arm, _gripper, FieldSide.LEFT, 0, _boxingGloves, false);
            center = Autos.scoreCenterNoMobility(_driveTrain, _arm, _gripper, FieldSide.LEFT, 0, _boxingGloves, false);
            right = Autos.sideAuto(_driveTrain, _arm, _gripper, _groundControl, FieldSide.LEFT, 0, 0, _boxingGloves, false);
            leftMid = Autos.sideJeremiahAuto(_driveTrain, _arm, _gripper, _groundControl, _boxingGloves, FieldSide.LEFT, 2, 2, false);
            centerDouble = Autos.scoreCenterExperimental(_driveTrain, _arm, _gripper, _groundControl, FieldSide.LEFT, 0, _boxingGloves, false);
            rightMid = Autos.sideJeremiahAuto(_driveTrain, _arm, _gripper, _groundControl, _boxingGloves, FieldSide.LEFT, 0, 0, false);
            
            leftPunching = Autos.sideAuto(_driveTrain, _arm, _gripper, _groundControl, FieldSide.LEFT, 2, 2, _boxingGloves, true);
            centerPunching = Autos.scoreCenter(_driveTrain, _arm, _gripper, FieldSide.LEFT, 0, _boxingGloves, true);
            rightPunching = Autos.sideAuto(_driveTrain, _arm, _gripper, _groundControl, FieldSide.LEFT, 0, 0, _boxingGloves, true);
            leftMidPunching = Autos.sideJeremiahAuto(_driveTrain, _arm, _gripper, _groundControl, _boxingGloves, FieldSide.LEFT, 2, 2, true);
            centerDoublePunching = Autos.scoreCenterExperimental(_driveTrain, _arm, _gripper, _groundControl, FieldSide.LEFT, 0, _boxingGloves, true);
            rightMidPunching = Autos.sideJeremiahAuto(_driveTrain, _arm, _gripper, _groundControl, _boxingGloves, FieldSide.LEFT, 0, 0, true);
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
            case("Center"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 1, 0));
                return center;
            case("Right"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 0, 0));
                // return Autos.bottomAutoThree(_driveTrain, FieldSide.LEFT, 0, 0);
                return right;
            case("Left"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 2, 2));
                return left;
            case("Center score double"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 1, 0));
                return centerDouble;
            case("Right score mid"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 0, 0));
                return rightMid;
            case("Left score mid"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 2, 2));
                return leftMid;
            case("Punching Center"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 1, 0));
                return centerPunching;
            case("Punching Right"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 0, 0));
                // return Autos.bottomAutoThree(_driveTrain, FieldSide.LEFT, 0, 0);
                return rightPunching;
            case("Punching Left"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 2, 2));
                return leftPunching;
            case("Punching Center score double"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 1, 0));
                return centerDoublePunching;
            case("Punching Right score mid"):
                _driveTrain.resetGyro();
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 0, 0));
                return rightMidPunching;
            case("Punching Left score mid"):
                _driveTrain.resetOdometry(FieldPoses.getScoringPose(FieldSide.LEFT, 2, 2));
                return leftMidPunching;
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