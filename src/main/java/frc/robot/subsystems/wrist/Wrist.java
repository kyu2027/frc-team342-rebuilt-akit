// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.wrist.WristIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Wrist {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs;

  private final Alert wristDisconnectedAlert;

  public Wrist(WristIO io) {
    this.io = io;
    inputs = new WristIOInputsAutoLogged();

    wristDisconnectedAlert = new Alert("Wrist motor is disconnected.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Wrist", inputs);

    wristDisconnectedAlert.set(!inputs.wristConnected);
  }

  public boolean getWristManual() {
    return inputs.wristManual;
  }

  public double getWristPosition() {
    return inputs.wristPosition;
  }

  public double getWristGoal() {
    return inputs.wristGoal;
  }

  public double getWristVelocity() {
    return inputs.wristVelocity;
  }

  public double getWristAppliedVoltage() {
    return inputs.wristAppliedVoltage;
  }

  public double getWristSupplyCurrentAmps() {
    return inputs.wristSupplyCurrentAmps;
  }
}
