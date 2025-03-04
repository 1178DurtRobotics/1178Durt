// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Constants.ModluleConstant.Drive;

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
    public static final int kOperatorControllerPort = 1;
    public static final double kCubic = 0.95;
    public static final double kLinear = 0.05;
    public static final double kDeadband = 0.10;
  }

  public static class CurrentLimit{
    public static final int kDrive = 40;
    public static final int kTurn = 20;
  }

  public static class DriveConstants{
    public static final double kWidth = 22.0/39.37;
    public static final double kLength = 27.0/39.37;

    public static final double kMaxSpeed = Drive.kMaxVel;
    public static final double kMaxAngularSpeed = 2*Math.PI;

    public static class FrontLeft{
      public static final int kDriveID = 1;
      public static final int kTurnID = 2;
      public static final int kEncoderID = 0;
      public static final double m_offset = 1.740;
      public static final Translation2d kPosition = new Translation2d(kLength/2.0,kWidth/2.0);
    }
    public static class FrontRight{
      public static final int kDriveID = 7;
      public static final int kTurnID = 8;
      public static final int kEncoderID = 3;
      public static final double m_offset = 5.359;
      public static final Translation2d kPosition = new Translation2d(kLength/2.0,-kWidth/2.0);
    }
    public static class RearLeft{
      public static final int kDriveID = 3;
      public static final int kTurnID = 4;
      public static final int kEncoderID = 1;
      public static final double m_offset = 2.602;
      public static final Translation2d kPosition = new Translation2d(-kLength/2.0,kWidth/2.0);
    }
    public static class RearRight{
      public static final int kDriveID = 5;
      public static final int kTurnID = 6;
      public static final int kEncoderID = 2;
      public static final double m_offset = 5.311;
      public static final Translation2d kPosition = new Translation2d(-kLength/2.0,-kWidth/2.0);
    }

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(FrontLeft.kPosition, FrontRight.kPosition, RearLeft.kPosition, RearRight.kPosition);

  }

  public static class ModluleConstant {
    public static class Drive{
      //                                      Pinion: 12T    Output: 18T    Bevels
      public static final int kPinion = 12;
      public static final int kOutput = 18;
      public static final double kGearRatio = (36.0/(double)kPinion) * ((double)kOutput/24.0) * (45.0/15.0);
      public static final double kWheelDiameter = 0.0985;
      public static final double kToMeters = (1.0/kGearRatio) * kWheelDiameter * Math.PI;
      public static final double kToRots = 1 / kToMeters;
      public static final double kMaxVel = 86.72 * kToMeters;
      public static final double kFF = 1.0/kMaxVel;
    }
    public static class Turn{
      public static final double kTurnP = 0.8;
    }
  }

  public static class GlobalConstants{
    public static final double kVoltComp = 11.0;
  }

  public static double kVoltComp;
}
