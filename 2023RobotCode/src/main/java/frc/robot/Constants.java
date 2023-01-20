// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class TurnToAngleConstants {
    public static final double kp = .0065;
    public static final double ki = .01;
    public static final double kd = 0.0;
    public static final double IRange = 5; //range, in degrees, in which the integral will start accumulating
    public static final double base = .27;
    public static final double maxEndVelocity = .1; //in degrees per 20ms, .1 converts to 5 deg/s
  }

  public static class BalanceConstants {
    public static final double kp = .0065;
    public static final double ki = .01;
    public static final double kd = 0.0;
    public static final double IRange = 5; //range, in degrees, in which the integral will start accumulating
    public static final double base = .27;
  }
}