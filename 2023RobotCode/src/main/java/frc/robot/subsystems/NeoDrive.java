// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.CompNeoBalanceConstants;
import frc.robot.Constants.CompNeoLockPositionConstants;
import frc.robot.Constants.CompNeoTurnToAngleConstants;
import frc.robot.Constants.PracticeNeoBalanceConstants;
import frc.robot.Constants.PracticeNeoTurnToAngleConstants;

/** Add your docs here. */
public class NeoDrive extends DriveTrain {
    //Speed Controllers
    private final CANSparkMax _leftLeadMotor;
    private final CANSparkMax _leftFollowMotor;
    private final CANSparkMax _rightLeadMotor;
    private final CANSparkMax _rightFollowMotor;
    private final DifferentialDrive _drive;

    private final RelativeEncoder _leftEncoder;
    private final RelativeEncoder _rightEncoder;

    private final double DISTANCE_PER_REVOLUTION;

    private boolean _isComp;
    private final Arm _arm;

    public NeoDrive(Arm arm, boolean isComp){
        super();
        _arm = arm;
        _isComp = isComp;

        //Assign robot-specific values
        if(_isComp){//Competition bot
            _leftLeadMotor = new CANSparkMax(3, MotorType.kBrushless);
            _leftFollowMotor = new CANSparkMax(4, MotorType.kBrushless);
            _rightLeadMotor = new CANSparkMax(1, MotorType.kBrushless);
            _rightFollowMotor = new CANSparkMax(2, MotorType.kBrushless);
            DISTANCE_PER_REVOLUTION = 0.04497067677447947292659295854798;
        } else {//Practice bot
            _leftLeadMotor = new CANSparkMax(5, MotorType.kBrushless);
            _leftFollowMotor = new CANSparkMax(8, MotorType.kBrushless);
            _rightLeadMotor = new CANSparkMax(7, MotorType.kBrushless);
            _rightFollowMotor = new CANSparkMax(6, MotorType.kBrushless);
            DISTANCE_PER_REVOLUTION = 0.04470389546284635574229691876751;
        }

        //Finish setting up differential drive and encoders
        _drive = new DifferentialDrive(_leftLeadMotor, _rightLeadMotor);
        _leftEncoder = _leftLeadMotor.getEncoder();
        _rightEncoder = _rightLeadMotor.getEncoder();

        _leftLeadMotor.setSmartCurrentLimit(40);
        _leftFollowMotor.setSmartCurrentLimit(40);
        _rightLeadMotor.setSmartCurrentLimit(40);
        _rightFollowMotor.setSmartCurrentLimit(40);
        
        
        //Set lead and follow motors
        _leftFollowMotor.follow(_leftLeadMotor);
        _rightFollowMotor.follow(_rightLeadMotor);

        //Set inversion
        if(_isComp){
            //Left side needs inverted
            _leftLeadMotor.setInverted(true);
            _rightLeadMotor.setInverted(false);
        } else{
            //Right side needs inverted
            _leftLeadMotor.setInverted(false);
            _rightLeadMotor.setInverted(true);
        }

        //Set distance per revolution for the encoders
        _leftEncoder.setPositionConversionFactor(DISTANCE_PER_REVOLUTION);
        _rightEncoder.setPositionConversionFactor(DISTANCE_PER_REVOLUTION);
        _leftEncoder.setVelocityConversionFactor(DISTANCE_PER_REVOLUTION/60.0);
        _rightEncoder.setVelocityConversionFactor(DISTANCE_PER_REVOLUTION/60.0);
        
    }

    /**
     * A multiplier to limit speed of drive based on the arm extension
     * @param minScale Smallest scale for when arm is fully extended
     * @param maxScale Largest scale for when arm is fully retracted
     * @return The scale, a value between minScale and maxScale
     */
    private double getDriveScale(double minScale, double maxScale) {
        // staged scale
        // if (_arm.getExtension() > .9) {
        //     return .05;
        // } else if (_arm.getExtension() > .5) {
        //     return .5;
        // } else {
        //     return 1;
        // }

        // continuous scale (visual here https://www.desmos.com/calculator/wkwibifzxw)
        return Math.max(minScale, Math.min(maxScale, (minScale + 1.1*Math.pow(1 - _arm.getExtension(), .7) * (maxScale - minScale))));
    }

    @Override
    public void tankDriveVolts (double leftVolts, double rightVolts) {
        if(_arm.isExtended()) {
            leftVolts = Math.signum(leftVolts) * Math.min(Math.abs(leftVolts), 12 * getDriveScale(.08, 1));
            rightVolts = Math.signum(rightVolts) * Math.min(Math.abs(rightVolts), 12 * getDriveScale(.08, 1));
        }
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
        speed = Math.signum(speed) * Math.min(Math.abs(speed), getDriveScale(.22, 1));
        steer = Math.signum(steer) * Math.min(Math.abs(steer), getDriveScale(.22, 1));

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
        return -getPitch(); // might be wrong, other bot is weird
    }

    @Override
    public double getTurnToAnglekP() {if(_isComp){return CompNeoTurnToAngleConstants.kp;} return PracticeNeoTurnToAngleConstants.kp;}
    @Override
    public double getTurnToAnglekI() {if(_isComp){return CompNeoTurnToAngleConstants.ki;} return PracticeNeoTurnToAngleConstants.ki;}
    @Override
    public double getTurnToAngleIRange() {if(_isComp){return CompNeoTurnToAngleConstants.IRange;} return PracticeNeoTurnToAngleConstants.IRange;}
    @Override
    public double getTurnToAnglekD() {if(_isComp){return CompNeoTurnToAngleConstants.kd;} return PracticeNeoTurnToAngleConstants.kd;}
    @Override
    public double getTurnToAngleBase() {if(_isComp){return CompNeoTurnToAngleConstants.base;} return PracticeNeoTurnToAngleConstants.base;}
    @Override
    public double getTurnToAngleMaxEndVelocity() {if(_isComp){return CompNeoTurnToAngleConstants.maxEndVelocity;} return PracticeNeoTurnToAngleConstants.maxEndVelocity;}
    @Override
    public double getTurnToAngleMaxAcceleration() {if(_isComp){return CompNeoTurnToAngleConstants.maxAccel;} return PracticeNeoTurnToAngleConstants.maxAccel;}
  
    @Override
    public double getBalancekP() {if(_isComp){return CompNeoBalanceConstants.kp;} return PracticeNeoBalanceConstants.kp;}
    @Override
    public double getBalancekI() {if(_isComp){return CompNeoBalanceConstants.ki;} return PracticeNeoBalanceConstants.ki;}
    @Override
    public double getBalanceIRange() {if(_isComp){return CompNeoBalanceConstants.IRange;} return PracticeNeoBalanceConstants.IRange;}
    @Override
    public double getBalancekD() {if(_isComp){return CompNeoBalanceConstants.kd;} return PracticeNeoBalanceConstants.kd;}
    @Override
    public double getBalanceBase() {if(_isComp){return CompNeoBalanceConstants.base;} return PracticeNeoBalanceConstants.base;}

    @Override
    public double getLockPositionkP() { return CompNeoLockPositionConstants.kp; }
    @Override
    public double getLockPositionkI() { return CompNeoLockPositionConstants.ki; }
    @Override
    public double getLockPositionIRange() { return CompNeoLockPositionConstants.IRange; }
    @Override
    public double getLockPositionkD() { return CompNeoLockPositionConstants.kd; }
    @Override
    public double getLockPositionBase() { return CompNeoLockPositionConstants.base; }

}
