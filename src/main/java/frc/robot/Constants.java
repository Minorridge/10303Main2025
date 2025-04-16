// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class AprilTagConstants {
        // Conversion factor
        // private static final double INCHES_TO_METERS = Units.inchesToMeters(1.0); // Defined if not elsewhere

        // --- AprilTag Field Poses (Speaker Tags Only) ---
        // Assumes Blue Alliance Origin, X towards Red Alliance, Y towards left
        // ** VERIFY these coordinates and rotations match the official field layout **

        // ---- Blue Alliance reef ----
        public static final Pose2d TAG_6_POSE = new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(300.0));
        public static final Pose2d TAG_7_POSE = new Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(0.0));   // Blue Center Speaker
        public static final Pose2d TAG_8_POSE = new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60.0));
        public static final Pose2d TAG_9_POSE = new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(120.0));
        public static final Pose2d TAG_10_POSE = new Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(180.0)); // Blue Center Stage-facing side
        public static final Pose2d TAG_11_POSE = new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(240.0));

        // ---- Red Alliance reef-
        public static final Pose2d TAG_17_POSE = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(240.0)); // Verify Z-Rot (using symmetry)
        public static final Pose2d TAG_18_POSE = new Pose2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(180.0)); // Red Center Speaker
        public static final Pose2d TAG_19_POSE = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(120.0));
        public static final Pose2d TAG_20_POSE = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60.0));
        public static final Pose2d TAG_21_POSE = new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(0.0));   // Red Center Stage-facing side? Verify Z-Rot
        public static final Pose2d TAG_22_POSE = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(300.0));


        // Map to easily retrieve poses by ID (Only Speaker Tags)
        public static final Map<Integer, Pose2d> TAG_FIELD_POSES = Map.ofEntries(
            // Blue Speaker
            Map.entry(6, TAG_6_POSE),
            Map.entry(7, TAG_7_POSE),
            Map.entry(8, TAG_8_POSE),
            Map.entry(9, TAG_9_POSE),
            Map.entry(10, TAG_10_POSE),
            Map.entry(11, TAG_11_POSE),
            // Red Speaker
            Map.entry(17, TAG_17_POSE),
            Map.entry(18, TAG_18_POSE),
            Map.entry(19, TAG_19_POSE),
            Map.entry(20, TAG_20_POSE),
            Map.entry(21, TAG_21_POSE),
            Map.entry(22, TAG_22_POSE)
        );
    }
  public static final class CoralSubsystemConstants {
    public static final int kElevatorMotorCanId = 10;
    public static final int kArmMotorCanId = 11;
    public static final int kIntakeMotorCanId = 12;

    public static final class ElevatorSetpoints {
      public static final int kFeederStation = 0;
      public static final int kLevel1 = 0;
      public static final int kLevel2 = 0;
      public static final int kLevel3 = 100;
      public static final int kLevel4 = 100;
    }

    public static final class ArmSetpoints {
      public static final double kFeederStation = -10;
      public static final double kLevel1 = 0;
      public static final double kLevel2 = -42;
      public static final double kLevel3 = -42;
      public static final double kLevel4 = -40;
    }

    public static final class IntakeSetpoints {
      public static final double kForward = 0.5;
      public static final double kReverse = -0.5;
    }
  }

  public static final class AlgaeSubsystemConstants {
    public static final int kIntakeMotorCanId = 13;
    public static final int kPivotMotorCanId = 14;

    public static final class ArmSetpoints {
      public static final double kStow = 0;
      public static final double kHold = -4.5;
      public static final double kDown = -8.5;
    }

    public static final class IntakeSetpoints {
      public static final double kForward = 0.6;
      public static final double kReverse = -0.6;
      public static final double kHold = 0.26;
    }
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
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
    public static final int kRearLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 5;
    public static final int kRearRightDrivingCanId = 3;

    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 4;

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
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
    public static final double kTriggerButtonThreshold = 0.2;
  }

  public static final class AutoConstants {
    public static final TrapezoidProfile.Constraints kXControllerConstraints =
        new TrapezoidProfile.Constraints(
            DriveConstants.kMaxSpeedMetersPerSecond * 0.8, // Max velocity (e.g., 80% of robot max)
            DriveConstants.kMaxSpeedMetersPerSecond * 1.5); // Max acceleration (tune this)
    public static final TrapezoidProfile.Constraints kYControllerConstraints =
        new TrapezoidProfile.Constraints(
            DriveConstants.kMaxSpeedMetersPerSecond * 0.8, // Max velocity
            DriveConstants.kMaxSpeedMetersPerSecond * 1.5); // Max acceleration

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class SimulationRobotConstants {
    public static final double kPixelsPerMeter = 20;

    public static final double kElevatorGearing = 25; // 25:1
    public static final double kCarriageMass =
        4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
    public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
    public static final double kMinElevatorHeightMeters = 0.922; // m
    public static final double kMaxElevatorHeightMeters = 1.62; // m

    public static final double kArmReduction = 52.0 / 28.0; // or 13.0 / 7.0 or 1.857142857;
    public static final double kArmLength = 0.433; // m
    public static final double kArmMass = 4.3; // Kg
    public static final double kMinAngleRads =
        Units.degreesToRadians(-50.1); // -50.1 deg from horiz
    public static final double kMaxAngleRads =
        Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

    public static final double kIntakeReduction = 135; // 135:1
    public static final double kIntakeLength = 0.4032262; // m
    public static final double kIntakeMass = 5.8738; // Kg
    public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
    public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
    public static final double kIntakeShortBarLength = 0.1524;
    public static final double kIntakeLongBarLength = 0.3048;
    public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
  }
}
