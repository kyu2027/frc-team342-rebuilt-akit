// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Roller extends SubsystemBase{
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs;

  private final Alert rollerDisconnectedAlert;

  public Roller(RollerIO io) {
    this.io = io;
    inputs = new RollerIOInputsAutoLogged();

    rollerDisconnectedAlert = new Alert("Roller motor is disconnected.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);

    rollerDisconnectedAlert.set(!inputs.rollerConnected);
  }
}
