// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.frontLeftCANCoderZeroRotation;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = Units.feetToMeters(20);
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(22.5);
  public static final double wheelBase = Units.inchesToMeters(22.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final double frontLeftCANCoderZeroRotation = -0.022216796875;
  public static final double frontRightCANCoderZeroRotation = -0.3671875;
  public static final double backLeftCANCoderZeroRotation = -0.330810546875;
  public static final double backRightCANCoderZeroRotation = -0.251708984375;

  public static final Rotation2d frontLeftZeroRotation =
      new Rotation2d(2 * Math.PI * frontLeftCANCoderZeroRotation); // Radians
  public static final Rotation2d frontRightZeroRotation =
      new Rotation2d(2 * Math.PI * frontRightCANCoderZeroRotation); // Radians
  public static final Rotation2d backLeftZeroRotation =
      new Rotation2d(2 * Math.PI * backLeftCANCoderZeroRotation); // Radians
  public static final Rotation2d backRightZeroRotation =
      new Rotation2d(2 * Math.PI * backRightCANCoderZeroRotation); // Radians

  // Device CAN IDs
  public static final int frontLeftDriveCanId = 1;
  public static final int frontRightDriveCanId = 2;
  public static final int backLeftDriveCanId = 3;
  public static final int backRightDriveCanId = 4;

  public static final int frontLeftTurnCanId = 5;
  public static final int frontRightTurnCanId = 6;
  public static final int backLeftTurnCanId = 7;
  public static final int backRightTurnCanId = 8;

  public static final int frontLeftCANCoderID = 9;
  public static final int frontRightCANCoderID = 10;
  public static final int backLeftCANCoderID = 11;
  public static final int backRightCANCoderID = 12;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 60;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2.0);
  public static final double driveGearRatio = 6.03;
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      wheelRadiusMeters * Math.PI / driveGearRatio; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      driveEncoderPositionFactor / 60; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 60;
  public static final double turnMotorGearRatio = 26.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / turnMotorGearRatio; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor =
      turnEncoderPositionFactor / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveGearRatio),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
