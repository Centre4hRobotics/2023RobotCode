// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/** Add your docs here. */
public class FalconDrive extends DriveTrain {
    //Speed Controllers
    private final WPI_TalonFX _leftLeadMotor = new WPI_TalonFX(4);
    private final WPI_TalonFX _leftFollowMotor = new WPI_TalonFX(3);
    private final WPI_TalonFX _rightLeadMotor = new WPI_TalonFX(2);
    private final WPI_TalonFX _rightFollowMotor = new WPI_TalonFX(1);
    private final DifferentialDrive _drive = new DifferentialDrive(_leftLeadMotor, _rightLeadMotor);

    private final double DISTANCE_PER_REVOLUTION = 0.000021828073956;

    public FalconDrive() {
        super();

        
        // _leftLeadMotor.configFactoryDefault();
        // _leftFollowMotor.configFactoryDefault();
        // _rightLeadMotor.configFactoryDefault();
        // _rightFollowMotor.configFactoryDefault();

        //Set current limits
        _leftLeadMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 50, 0.5));
        _leftFollowMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 50, 0.5));
        _rightLeadMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 50, 0.5));
        _rightFollowMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 50, 0.5));


        //Set lead and follow motors
        _leftFollowMotor.follow(_leftLeadMotor);
        _rightFollowMotor.follow(_rightLeadMotor);

        //Right side needs inverted
        _leftLeadMotor.setInverted(false);
        _rightLeadMotor.setInverted(true);
        _leftFollowMotor.setInverted(InvertType.FollowMaster);
        _rightFollowMotor.setInverted(InvertType.FollowMaster);



        _leftLeadMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        _rightLeadMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        _leftFollowMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        _rightFollowMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        
    }

    @Override
    public void tankDriveVolts (double leftVolts, double rightVolts) {
        _leftLeadMotor.setVoltage(leftVolts);
        _rightLeadMotor.setVoltage(rightVolts);
    
        _drive.feed();//makes sure differencial drive knows something bad hasn't happened
    }

    /**
     * Drive the robot with basic arcade steering
     * @param speed Forward or backward speed [-1, 1]
     * @param steer Steering [-1, 1]
     */
    @Override
    public void arcadeDrive(double speed, double steer) {
        _drive.arcadeDrive(speed, steer);
        _drive.feed();//makes sure differencial drive knows something bad hasn't happened
    }

    @Override
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(_leftLeadMotor.getSelectedSensorVelocity()*DISTANCE_PER_REVOLUTION*10, _rightLeadMotor.getSelectedSensorVelocity()*DISTANCE_PER_REVOLUTION*10);
    }

    /**
     * Get distance in meters from the left encoder
     * @return distance in meters traveled since last reset
     */
    @Override
    public double getLeftEncoder() {
        return _leftLeadMotor.getSelectedSensorPosition()*DISTANCE_PER_REVOLUTION;
    }

    /**
     * Get distance in meters from the right encoder
     * @return distance in meters traveled since last reset
     */
    @Override
    public double getRightEncoder() {
        return _rightLeadMotor.getSelectedSensorPosition()*DISTANCE_PER_REVOLUTION;
    }

    @Override
    public void resetEncoders() {
        _leftLeadMotor.setSelectedSensorPosition(0);
        _rightLeadMotor.setSelectedSensorPosition(0);
    }

    public double getRobotPitch() {
        return getPitch();
    }

}