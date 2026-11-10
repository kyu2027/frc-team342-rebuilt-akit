// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Wrist extends SubsystemBase{
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs;

  private final SysIdRoutine wristSysId;

  private final Alert wristDisconnectedAlert;

  public Wrist(WristIO io) {
    this.io = io;
    inputs = new WristIOInputsAutoLogged();

    
    wristSysId = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(1).per(Second),
          Volts.of(6),
          Seconds.of(30),
          (state) -> Logger.recordOutput("Wrist", state.toString())
        ),
      new SysIdRoutine.Mechanism(
        (volts) -> io.setVoltage(volts.in(Volts)), 
        null, 
        this));

    wristDisconnectedAlert = new Alert("Wrist motor is disconnected.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

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
