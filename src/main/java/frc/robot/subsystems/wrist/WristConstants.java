// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/** Add your docs here. */
public class WristConstants {
  public static final int wristMotorID = 14;

  public static final double wristMotorMOI = 0.000027;
  public static final double wristGearRatio = 25.0;
  public static final double wristPositionConversionFactor =
      (2 * Math.PI) / wristGearRatio; // Rotations -> Radians

  public static final double wristAllowedError = 0.025; // Radians

  public static final double wristExtendedPosition = 5.7065; // Radians
  public static final double wristStartingPosition = 0.01; // Radians
  public static final double wristMiddlePosition = 2.182; // Radians

  public static final double[] wristProfiledPIDValues = {0.25, 0.0, 0.2}; // PID
  public static final double[] wristFeedforwardValues = {0.0, 0.0, 0.0, 0.0}; // SGVA

  public static final Constraints wristProfiledPIDConstraints = new Constraints(Math.PI * 2, Math.PI * 4);

  public static final int wristCurrentLimit = 30; // Amps
}
