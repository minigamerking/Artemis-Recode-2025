// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static class OperatorConstants
    {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final double kDeadband = 0.07;
    }

    public static class RobotConstants
    {
        public static final double chassisLength = Units.inchesToMeters(20);
        public static final double chassisWidth = Units.inchesToMeters(21.25);
        public static final double kMetersPerSec = Units.feetToMeters(20);
        public static final double kRadiansPerSec = Units.rotationsToRadians(2);
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 300 / 49;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
        public static final double kDriveEncoderRot2Meter = (Math.PI * kWheelDiameterMeters) / kDriveMotorGearRatio;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * 180;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.0;
        public static final double kDTurning = 0.01;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(21.25);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(20);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kBackLeftDriveMotorPort = 5;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackRightDriveMotorPort = 7;

        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kBackRightTurningMotorPort = 8;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
        public static final int kBackRightDriveAbsoluteEncoderPort = 12;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        public static final double kFrontLeftDriveAbsoluteEncoderOffset = -1.270128;
        public static final double kBackLeftDriveAbsoluteEncoderOffset = -1.244053;
        public static final double kFrontRightDriveAbsoluteEncoderOffset = -0.1702743;
        public static final double kBackRightDriveAbsoluteEncoderOffset = 1.935884;

        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 3.63;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final double kPhysicalMaxAngularSpeedDegreesPerSecond = 2 * 2 * 180;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.5;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedDegreesPerSecond = //
                kPhysicalMaxAngularSpeedDegreesPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 4;
    }

    public static final class ArmConstants {
        public static final int kArmMinAngleDegrees = 15;
        public static final int kArmMaxAngleDegrees = 110;
        public static final int kArmCalibrationAngleDegrees = 90;

        public static final int kArmLeftMotorPort = 17;
        public static final int kArmRightMotorPort = 16;

        public static final int kArmLeftShooterPort = 14;
        public static final int kArmRightShooterPort = 15;
        
        public static final int kArmLeftIntakePort = 18;
        public static final int kArmRightIntakePort = 19;

        public static final int kShooterBeamBreakPort = 0;

        public static final double kArmShooterSpeed = 1;
        public static final double kArmIntakeSpeed = 1;
    }
}
