// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean wristConnected = false;

    public double wristPosition = 0.0;
    public double wristGoal = 0.0;
    public double wristVelocity = 0.0;

    public double wristAppliedVoltage = 0.0;
    public double wristSupplyCurrentAmps = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  /** Sets the voltage of the wrist motor */
  public default void setVoltage(double voltage) {}

  /** Move wrist to set position */
  public default void wristToPosition(double goal) {}

  /** Stop the wrist */
  public default void stopWrist() {}

  /** Reset the wrist encoder to 0.0 */
  public default void resetWristEncoder() {}
}
