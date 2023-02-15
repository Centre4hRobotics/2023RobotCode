// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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

  public static class Colors {
    public static final String ANSI_RESET = "\u001B[0m";
    public static final String ANSI_BLACK = "\u001B[30m";
    public static final String ANSI_RED = "\u001B[31m";
    public static final String ANSI_GREEN = "\u001B[32m";
    public static final String ANSI_YELLOW = "\u001B[33m";
    public static final String ANSI_BLUE = "\u001B[34m";
    public static final String ANSI_PURPLE = "\u001B[35m";
    public static final String ANSI_CYAN = "\u001B[36m";
    public static final String ANSI_WHITE = "\u001B[37m";
  }
  
  //robot characterization values
    
    public static final double ksVolts = .10737;
    public static final double kvVoltsSecondsPerMeter = 2.3429; //Falcon practice
    public static final double kaVoltsSecondsSquaredPerMeter = .19912;
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
    public static final double kd = 0.00025;
    public static final double IRange = 5; //range, in degrees, in which the integral will start accumulating
    public static final double base = .27;
    public static final double maxEndVelocity = .1; //in degrees per 20ms, .1 converts to 5 deg/s
    public static final double maxAccel = .3; //per 20 ms, from 0-1. .2 means you can go 0-full in .1s
    
  }

  public static class PracticeNeoBalanceConstants {
    public static final double kp = .006;
    public static final double ki = .01;
    public static final double kd = .0008;
    public static final double IRange = 5; //range, in degrees, in which the integral will start accumulating
    public static final double base = .27;
  }

  public static class CompNeoTurnToAngleConstants {
    public static final double kp = .0065;
    public static final double ki = .01;
    public static final double kd = 0.00025;
    public static final double IRange = 5; //range, in degrees, in which the integral will start accumulating
    public static final double base = .27;
    public static final double maxEndVelocity = .1; //in degrees per 20ms, .1 converts to 5 deg/s
    public static final double maxAccel = .3; //per 20 ms, from 0-1. .2 means you can go 0-full in .1s
  }

  public static class CompNeoBalanceConstants {
    public static final double kp = .006;
    public static final double ki = .01;
    public static final double kd = .0008;
    public static final double IRange = 5; //range, in degrees, in which the integral will start accumulating
    public static final double base = .27;
  }

  public static class FalconTurnToAngleConstants {
    public static final double kp = .0065;
    public static final double ki = .01;
    public static final double kd = 0.00025;
    public static final double IRange = 5; //range, in degrees, in which the integral will start accumulating
    public static final double base = .27;
    public static final double maxEndVelocity = .1; //in degrees per 20ms, .1 converts to 5 deg/s
    public static final double maxAccel = .3; //per 20 ms, from 0-1. .2 means you can go 0-full in .1s
  }

  public static class FalconBalanceConstants {
    public static final double kp = .006;
    public static final double ki = .00;
    public static final double kd = .0008;
    public static final double IRange = 5; //range, in degrees, in which the integral will start accumulating
    public static final double base = .27;
  }

  public static class FalconSize {
    public static final double length = 1;
    public static final double width = .85;
  }

  public static class AprilTagPoses{
    private static final Pose3d[] Poses = {
      new Pose3d(15.51356, 1.071626, 0.462788, new Rotation3d(0, 0, Math.PI)), //tag 1
      new Pose3d(15.51356, 2.748026, 0.462788, new Rotation3d(0, 0, Math.PI)),
      new Pose3d(15.51356, 4.424426, 0.462788, new Rotation3d(0, 0, Math.PI)),
      new Pose3d(16.17878, 6.749796, 0.695452, new Rotation3d(0, 0, Math.PI)), //I'm not confident on this y value
      new Pose3d(0.36195, 6.749796, 0.695452, new Rotation3d(0, 0, 0)), //I'm not confident on this y value
      new Pose3d(1.02743, 4.424426, 0.462788, new Rotation3d(0, 0, 0)),
      new Pose3d(1.02743, 2.748026, 0.462788, new Rotation3d(0, 0, 0)),
      new Pose3d(1.02743, 1.071626, 0.462788, new Rotation3d(0, 0, 0)), //tag 8
    };

    public static final Pose3d getPose(int tagID){
      //Return null if tag does not exist
      if (tagID < 1 || tagID > 8){
        return null;
      }

      return Poses[tagID-1];
    }
  }

  public static class CameraPoses{
    private static final Translation3d practiceBotTranslation = new Translation3d(.2, .025, -.47);//Center of robot in relation to camera
    private static final Rotation3d practiceBotRotation = new Rotation3d(0, 0, 0);

    public static final Transform3d getCameraPose() {
      return new Transform3d(practiceBotTranslation, practiceBotRotation);
    }
  }

  public static enum FieldSide {
    LEFT, RIGHT
  }

  public static class FieldPoses {
    private static final double leftScoringX = 1.377952756, rightScoringX = 15.16383033;
    private static final double[] yScoringPositions = {
      // y calculated from +- (inner node width + divider width)
      0.5079988824, 1.0668, 1.625601118, 
      2.184398882, 2.7432, 3.302001118, 
      3.860798882, 4.4196, 4.978401118
    };

    /**
     * Get pose of a scoring position
     * @param side Which side of the field, either LEFT (Blue) or RIGHT (Red)
     * @param grid Which grid of nodes 0-2, 0 being the left most when looking from the charging station
     * @param node Which node in the grid 0-2, 0 being the left most when looking from the charging station
     * @throws Exception
     */
    public static final Pose2d getScoringPose(FieldSide side, int grid, int node) throws Exception {
      double offset = 0;

      double x;
      Rotation2d rotation;
      switch (side) {
        case LEFT: 
          x = leftScoringX + FalconSize.length / 2 + offset;
          rotation = new Rotation2d(Math.PI);
          break;
        case RIGHT: 
          x = rightScoringX - FalconSize.length / 2 - offset;
          rotation = new Rotation2d(0);
          break;
        default: 
          throw new Exception("side isn't left or right?");
      }

      if (grid < 0 || grid > 2) {
        throw new Exception("grid should be between 0 and 2");
      }
      if (node < 0 || node > 2) {
        throw new Exception("node should be between 0 and 2");
      }
      
      double y = yScoringPositions[grid * 3 + node];

      return new Pose2d(x, y, rotation);
    }

    public static final Pose2d getOnChargingStationPose(FieldSide side) throws Exception {
      // not calculated, used pathweaver
      switch (side) {
        case LEFT: 
          return new Pose2d(2.9, 2.727, new Rotation2d(Math.PI));
        case RIGHT: 
          return new Pose2d(13.901, 2.727, new Rotation2d(0));
        default: 
          throw new Exception("side isn't left or right?");
      }
    }

    //Get position toward center of field that would work to drive off center of charging station
    public static final Pose2d getOffChargingStationPose(FieldSide side) throws Exception {
      // not calculated, used pathweaver
      switch (side) {
        case LEFT: 
          return new Pose2d(5.45, 2.727, new Rotation2d(Math.PI));
        case RIGHT: 
          return new Pose2d(11.05, 2.727, new Rotation2d(0));
        default: 
          throw new Exception("side isn't left or right?");
      }
    }
  }
}
