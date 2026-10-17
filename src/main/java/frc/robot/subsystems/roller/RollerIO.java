// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public static class RollerIOInputs {
    public boolean rollerConnected = false;

    public AngularVelocity rollerVelocity = RotationsPerSecond.zero();

    public double rollerAppliedVoltage = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
  }

  public default void updateInputs(RollerIOInputs inputs) {}

  /** Spin roller at set speed */
  public default void spinRoller(double speed) {}

  /** Set roller to given voltage */
  public default void setVoltage(double voltage) {}

  /** Stop the roller */
  public default void stopRoller() {}
}
