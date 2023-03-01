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
    
    // public static final double ksVolts = .10737;
    // public static final double kvVoltsSecondsPerMeter = 2.3429; //Falcon practice
    // public static final double kaVoltsSecondsSquaredPerMeter = .19912;
    public static final double ksVolts = 0.106;
    public static final double kvVoltsSecondsPerMeter = 2.7472; //competition
    public static final double kaVoltsSecondsSquaredPerMeter = 0.59873;

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

  public static class ArmConstants {
    public static final double retracted = 0;
    public static final double lowPosition = 26*ArmConstants.encoderTicksToMeters;
    public static final double middlePosition = 114.810*ArmConstants.encoderTicksToMeters;
    public static final double highPosition = 175*ArmConstants.encoderTicksToMeters;//was 184.6
    public static final double pickupPosition = 39*ArmConstants.encoderTicksToMeters; // was 48.5, 50.5
    public static final double maxExtention = 176*ArmConstants.encoderTicksToMeters; // was 199.6
    public static final double encoderTicksToMeters = 0.005855208;

  }

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
    public static final double kd = .0032;//Was .0008, .0015, semi-worked at .0025, then 0.003
    public static final double IRange = 7; //range, in degrees, in which the integral will start accumulating
    public static final double base = .27;
  }

  public static class CompNeoLockPositionConstants {
    public static final double kp = 50;
    public static final double ki = .1;
    public static final double kd = 0;
    public static final double IRange = 5; 
    public static final double base = 0;
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

  public static class FalconLockPositionConstants {
    public static final double kp = 50;
    public static final double ki = .1;
    public static final double kd = 0;
    public static final double IRange = 5; 
    public static final double base = 0;
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

    private static final double leftStagingX = leftScoringX + 5.6896, rightStagingX = rightScoringX - 5.6896;
    private static final double[] yStagingPositions = {
      0.92075, 0.92075 + 1.2192, 0.92075 + 2 * 1.2192, 0.92075 + 3 * 1.2192
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
          throw new IllegalArgumentException("side isn't left or right?");
      }

      if (grid < 0 || grid > 2) {
        throw new IllegalArgumentException("grid should be between 0 and 2");
      }
      if (node < 0 || node > 2) {
        throw new IllegalArgumentException("node should be between 0 and 2");
      }
      
      double y = yScoringPositions[grid * 3 + node];

      return new Pose2d(x, y, rotation);
    }

    /**
     * Get pose for being partially on charging station
     * @param side LEFT (Blue) or RIGHT (Red)
     * @param chargeSide INSIDE or OUTSIDE (inside is within community, closer to scoring grids)
     * @return
     * @throws Exception
     */
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
          throw new IllegalArgumentException("side isn't left or right");
      }
    }

    /**
     * Get Pose2d of game pieces near middle of the field
     * @param side LEFT (Blue) or RIGHT (RED)
     * @param position 0 is closest to edge of field, 3 is farthest
     * @return
     * @throws Exception
     */
    public static final Pose2d getStagingPose(FieldSide side, int position) throws Exception {
      double grabberOffset = .3;

      double x;
      Rotation2d rotation;
      switch (side) {
        case LEFT:
          x = leftStagingX - FalconSize.length / 2 - grabberOffset;
          rotation = new Rotation2d(Math.PI); // faces opposite way, turns in auto
          break;
        case RIGHT:
          x = rightStagingX + FalconSize.length / 2 + grabberOffset;
          rotation = new Rotation2d(0);
          break;
        default: 
          throw new IllegalArgumentException("side isn't left or right?");
      }

      assert 0 <= position && position <= 3;

      double y = yStagingPositions[position];

      return new Pose2d(x, y, rotation);
    }

    public static final Pose2d getStagingPose(FieldSide side, int position, double angle) throws Exception {
      double grabberOffset = .3;
  
      double x;
      Rotation2d rotation;
      switch (side) {
        case LEFT:
          x = leftStagingX - FalconSize.length / 2 - grabberOffset;
          rotation = new Rotation2d(angle); // faces opposite way, turns in auto
          break;
        case RIGHT:
          x = rightStagingX + FalconSize.length / 2 + grabberOffset;
          rotation = new Rotation2d(angle);
          break;
        default: 
          throw new IllegalArgumentException("side isn't left or right?");
      }
  
      assert 0 <= position && position <= 3;
  
      double y = yStagingPositions[position];
  
      return new Pose2d(x, y, rotation);
    }

    /**
     * Get a waypoint around charging station
     * @param side
     * @param isCloseToWall
     * @param isFacingGrid
     * @return pose for waypoint around charging station
     */
    public static final Pose2d getAvoidChargingStationPose(FieldSide side, boolean isCloseToWall, boolean isFacingGrid){
      double x;
      if (side == FieldSide.LEFT){
        x = 4.77;
      } else{
        x = 11.77;
      }

      double y;
      if (isCloseToWall){
        y = .7425;
      } else{
        y = 4.642;
      }

      Rotation2d rotation;
      if ((side == FieldSide.LEFT && isFacingGrid) || (side == FieldSide.RIGHT && !isFacingGrid)){
        rotation = new Rotation2d(Math.PI);
      } else{
        rotation = new Rotation2d(0);
      }

      return new Pose2d(x, y, rotation);
    }

    public static final Pose2d getAvoidGamePiecePose(FieldSide side, int grid) {
      double offset = .6;
      if(side==FieldSide.LEFT) {
        if(grid==0) {
          return new Pose2d(getTrueStagingPose(side, grid).getX()-offset, getTrueStagingPose(side, grid).getY()+offset, new Rotation2d(-(Math.PI*3)/4));
        }
        return new Pose2d(getTrueStagingPose(side, grid).getX()-offset, getTrueStagingPose(side, grid).getY()-offset, new Rotation2d((Math.PI*3)/4));
      }
      if(grid==0) {
        return new Pose2d(getTrueStagingPose(side, grid).getX()+offset, getTrueStagingPose(side, grid).getY()+offset, new Rotation2d(-(Math.PI)/4));
      }
      return new Pose2d(getTrueStagingPose(side, grid).getX()+offset, getTrueStagingPose(side, grid).getY()+offset, new Rotation2d((Math.PI)/4));
    }

    //No staging pose of field elements without offset
    public static final Pose2d getTrueStagingPose(FieldSide side, int position){
      double x;
      switch (side) {
        case LEFT:
          x = leftStagingX;
          break;
        case RIGHT:
          x = rightStagingX;
          break;
        default: 
          throw new IllegalArgumentException("side isn't left or right?");
      }
  
      double y = yStagingPositions[position];

      return new Pose2d(x, y, new Rotation2d(0));
    }
  }
}

