// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/** Add your docs here. */
public class NeoDrive extends DriveTrain {
    //Speed Controllers
    private final CANSparkMax _leftLeadMotor = new CANSparkMax(5, MotorType.kBrushless);
    private final CANSparkMax _leftFollowMotor = new CANSparkMax(8, MotorType.kBrushless);
    private final CANSparkMax _rightLeadMotor = new CANSparkMax(7, MotorType.kBrushless);
    private final CANSparkMax _rightFollowMotor = new CANSparkMax(6, MotorType.kBrushless);
    private final DifferentialDrive _drive = new DifferentialDrive(_leftLeadMotor, _rightLeadMotor);

    private final RelativeEncoder _leftEncoder = _leftLeadMotor.getEncoder();
    private final RelativeEncoder _rightEncoder = _rightLeadMotor.getEncoder();

    private final double DISTANCE_PER_REVOLUTION = 0.04470389546284635574229691876751;

    private boolean _isComp;

    public NeoDrive(boolean isComp){
        super();

        _isComp = isComp;
        
        //Set lead and follow motors
        _leftFollowMotor.follow(_leftLeadMotor);
        _rightFollowMotor.follow(_rightLeadMotor);

        //Right side needs inverted
        _leftLeadMotor.setInverted(false);
        _rightLeadMotor.setInverted(true);

        //Set distance per revolution for the encoders
        _leftEncoder.setPositionConversionFactor(DISTANCE_PER_REVOLUTION);
        _rightEncoder.setPositionConversionFactor(DISTANCE_PER_REVOLUTION);
        _leftEncoder.setVelocityConversionFactor(DISTANCE_PER_REVOLUTION/60.0);
        _rightEncoder.setVelocityConversionFactor(DISTANCE_PER_REVOLUTION/60.0);
        
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
        return new DifferentialDriveWheelSpeeds(_leftEncoder.getVelocity(), _rightEncoder.getVelocity());
    }

    /**
     * Get distance in meters from the left encoder
     * @return distance in meters traveled since last reset
     */
    @Override
    public double getLeftEncoder() {
        return _leftEncoder.getPosition();
    }

    /**
     * Get distance in meters from the right encoder
     * @return distance in meters traveled since last reset
     */
    @Override
    public double getRightEncoder() {
        return _rightEncoder.getPosition();
    }

    @Override
    public void resetEncoders() {
        _leftEncoder.setPosition(0);
        _rightEncoder.setPosition(0);
    }

    @Override
    public double getRobotPitch() {
        return getRoll();
    }

}
