// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.4;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // rad/sec

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    public static final double kWheelBase = Units.inchesToMeters(23.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kFrontRightDrivingCanId = 8;
    public static final int kRearRightDrivingCanId = 9;


    public static final int kFrontLeftTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 5;


    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kAbsoluteControllerPort = 5;
    public static final double kDriveDeadband = 0.05;
  }


public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;


    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;


    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularSpeedRadiansPerSecondSquared);
  }


  public static final class IntakeConstants {
    public static final int kLeftIntakeMotorCanId = 13;
    public static final int kRightIntakeMotorCanId = 15;


    public static final double kIntakeVelocity = 450;
    public static final double kShootTowerVelocity = 480;
    public static final double kFeedNeutralVelocity = 550;


    public static final double kIntakeP = 0.02;
    public static final double kIntakeI = 0.0;
    public static final double kIntakeD = 0.0005;
    public static final double kIntakeS = 0.175;
    public static final double kIntakeV = 0.017;
  }

    public static final class FeederConstants {
    public static final int kFeederMotorCanId = 12;
    public static final double kFeederMotorCurrentLimit = 40;
    public static double kFeederFowardSpeed = 0.6;
    public static double kFeederReverseSpeed = -0.6;
  }


  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }



  public static final class Targets {

    public static final double FIELD_LENGTH = 16.5405;
    public static final double FIELD_WIDTH = 8.0695;

    public static final double CENTER_Y =
        FIELD_WIDTH / 2.0;

    public static final Pose2d BLUE_TARGET =
        makeTarget(
            4.625,
            CENTER_Y);

    public static final Pose2d RED_TARGET =
        makeTarget(
            FIELD_LENGTH - 4.625,
            CENTER_Y);
}

private static Pose2d makeTarget(
        double x,
        double y) {

    return new Pose2d(
        x,
        y,
        Rotation2d.kZero);
}



}