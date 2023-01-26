// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  //robot characterization values
    
    public static final double ksVolts = 0.1418;
    public static final double kvVoltsSecondsPerMeter = 2.8433; //practice
    public static final double kaVoltsSecondsSquaredPerMeter = 0.27701;
    // public static final double ksVolts = 0.62303;
    // public static final double kvVoltsSecondsPerMeter = 2.364; //competition
    // public static final double kaVoltsSecondsSquaredPerMeter = 0.3286;

    //Porportional control in PID loop (need to tune this)
    public static final double kpDriveVel = 0.5;  //Characterization tool suggests 2.1325

    
    public static final double kTrackwidthMeters = 0.558; //Width of robot (between wheels)
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);


    //Tune these parameters to make the robot go faster
    public static final double kMaxSpeedMetersPerSecond = 3.;//can go up to ~3, reduced for testing
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;//set to one for testing



    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;//not final
    public static final double kRamseteZeta = 0.7;//not final

    // field positions
    public static final double fieldCenterX = 8.23;
    public static final double fieldCenterY = 4.115;


  public static class PracticeNeoTurnToAngleConstants {
    public static final double kp = .0065;
    public static final double ki = .01;
    public static final double kd = 0.0;
    public static final double IRange = 5; //range, in degrees, in which the integral will start accumulating
    public static final double base = .27;
    public static final double maxEndVelocity = .1; //in degrees per 20ms, .1 converts to 5 deg/s
    public static final double maxAccel = .3; //per 20 ms, from 0-1. .2 means you can go 0-full in .1s
    
  }

  public static class PracticeNeoBalanceConstants {
    public static final double kp = .0045;
    public static final double ki = .01;
    public static final double kd = .0005;
    public static final double IRange = 5; //range, in degrees, in which the integral will start accumulating
    public static final double base = .27;
  }

  public static class CompNeoTurnToAngleConstants {
    public static final double kp = .0065;
    public static final double ki = .01;
    public static final double kd = 0.0;
    public static final double IRange = 5; //range, in degrees, in which the integral will start accumulating
    public static final double base = .27;
    public static final double maxEndVelocity = .1; //in degrees per 20ms, .1 converts to 5 deg/s
    public static final double maxAccel = .3; //per 20 ms, from 0-1. .2 means you can go 0-full in .1s
  }

  public static class CompNeoBalanceConstants {
    public static final double kp = .0045;
    public static final double ki = .01;
    public static final double kd = .0005;
    public static final double IRange = 5; //range, in degrees, in which the integral will start accumulating
    public static final double base = .27;
  }

  public static class FalconTurnToAngleConstants {
    public static final double kp = .0065;
    public static final double ki = .01;
    public static final double kd = 0.0;
    public static final double IRange = 5; //range, in degrees, in which the integral will start accumulating
    public static final double base = .27;
    public static final double maxEndVelocity = .1; //in degrees per 20ms, .1 converts to 5 deg/s
    public static final double maxAccel = .3; //per 20 ms, from 0-1. .2 means you can go 0-full in .1s
  }

  public static class FalconBalanceConstants {
    public static final double kp = .0045;
    public static final double ki = .00;
    public static final double kd = .0005;
    public static final double IRange = 5; //range, in degrees, in which the integral will start accumulating
    public static final double base = .27;
  }
}
