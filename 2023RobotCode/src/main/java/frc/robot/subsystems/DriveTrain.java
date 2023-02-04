// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;




public abstract class DriveTrain extends SubsystemBase {

  //private final AHRS gyro = new AHRS(I2C.Port.kMXP);
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);  //Use SPI instead of I2C
  private final DifferentialDriveOdometry _odometry;

  /** Creates a new DriveTrain. */
  public DriveTrain() { 
    // _odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    _odometry = new DifferentialDriveOdometry(getRotation2d(), 0, 0);
  }

  @Override
  public void periodic() {

    //Odometry
    _odometry.update(getRotation2d(), getLeftEncoder(), getRightEncoder());

    //Get network table
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    //Write values to network table
    nt.getTable("Drive").getEntry("LeftEncoderDistance").setValue(getLeftEncoder());
    nt.getTable("Drive").getEntry("RightEncoderDistance").setValue(getRightEncoder());
    nt.getTable("Drive").getEntry("Odometry").setValue(getPose().toString());
    nt.getTable("Drive").getEntry("X").setValue(getPose().getX());
    nt.getTable("Drive").getEntry("Y").setValue(getPose().getY());

    nt.getTable("Gyro").getEntry("Yaw").setValue(getYaw());
    nt.getTable("Gyro").getEntry("Angle").setValue(getAngle());
    nt.getTable("Gyro").getEntry("Roll").setValue(getRoll());
    nt.getTable("Gyro").getEntry("Pitch").setValue(getPitch());
    nt.getTable("Gyro").getEntry("Rotation2d").setValue(getRotation2d().toString());

  } 

  
  

  public Pose2d getPose() {
    return _odometry.getPoseMeters();
  }

  /**
   * Return the heading of the robot
   * @return continuous robot heading in degrees
   */
  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  /**
   * Get gyro yaw
   * @return rotation around z-axis [-180, 180] in degrees
   */
  public double getYaw () {
    return -gyro.getYaw();
  }
  public double getRoll () {
    return gyro.getRoll();
  }
  public double getPitch () {
    return gyro.getPitch();
  }
  /**
   * Get gyro angle
   * @return return the yaw value (but continuous) in degrees
   */
  public double getAngle () {
    return -gyro.getAngle();  // negative to make counter-clockwise
  }

  public void resetOdometry(Pose2d pose) {
    //resetEncoders();
    _odometry.resetPosition(getRotation2d(), getLeftEncoder(), getRightEncoder(), pose);
  }

  public void resetGyro() {
    gyro.reset();
  }


  //Implement these abstract methods!
  public abstract void arcadeDrive(double speed, double steer);

  public abstract void tankDriveVolts (double leftVolts, double rightVolts);
  
  public abstract DifferentialDriveWheelSpeeds getWheelSpeeds();
  
  public abstract double getLeftEncoder();
  
  public abstract double getRightEncoder();

  public abstract void resetEncoders();

  public abstract double getRobotPitch();

  public abstract double getTurnToAnglekP();
  public abstract double getTurnToAnglekI();
  public abstract double getTurnToAngleIRange();
  public abstract double getTurnToAnglekD();
  public abstract double getTurnToAngleBase();
  public abstract double getTurnToAngleMaxEndVelocity();
  public abstract double getTurnToAngleMaxAcceleration();

  public abstract double getBalancekP();
  public abstract double getBalancekI();
  public abstract double getBalanceIRange();
  public abstract double getBalancekD();
  public abstract double getBalanceBase();
}