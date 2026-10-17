// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

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

  public static final double[] wristPIDValuesSlot0 = {0.165, 0.0, 0.0295};
  public static final double[] wristPIDValuesSlot1 = {0.2, 0.0, 0.25};

  public static final int wristCurrentLimit = 30; // Amps
}
