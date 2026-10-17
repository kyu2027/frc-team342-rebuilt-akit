// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static frc.robot.subsystems.wrist.WristConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class WristIOSpark implements WristIO {
  private final SparkFlex wristMotor;
  private final RelativeEncoder wristEncoder;

  private final SparkClosedLoopController wristController;

  private final Debouncer wristConnectedDebouncer;

  private boolean manual;

  public WristIOSpark() {
    wristMotor = new SparkFlex(wristMotorID, MotorType.kBrushless);
    wristEncoder = wristMotor.getEncoder();

    wristController = wristMotor.getClosedLoopController();

    wristConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

    manual = false;

    var wristConfig = new SparkFlexConfig();
    wristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(wristCurrentLimit);
    wristConfig.encoder.positionConversionFactor(wristPositionConversionFactor);
    wristConfig
        .closedLoop
        .allowedClosedLoopError(wristAllowedError, ClosedLoopSlot.kSlot0)
        .allowedClosedLoopError(wristAllowedError, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1)
        .positionWrappingEnabled(false)
        .pid(wristPIDValuesSlot0[0], wristPIDValuesSlot0[1], wristPIDValuesSlot0[2])
        .pid(wristPIDValuesSlot1[0], wristPIDValuesSlot1[1], wristPIDValuesSlot1[2]);

    tryUntilOk(
        wristMotor,
        5,
        () ->
            wristMotor.configure(
                wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(wristMotor, () -> wristEncoder.getPosition(), (value) -> inputs.wristPosition = value);
    ifOk(wristMotor, () -> wristEncoder.getVelocity(), (value) -> inputs.wristVelocity = value);
    ifOk(wristMotor, () -> wristController.getSetpoint(), (value) -> inputs.wristGoal = value);
    ifOk(
        wristMotor,
        () -> wristMotor.getAppliedOutput() * wristMotor.getBusVoltage(),
        (value) -> inputs.wristAppliedVoltage = value);
    ifOk(
        wristMotor,
        () -> wristMotor.getOutputCurrent(),
        (value) -> inputs.wristSupplyCurrentAmps = value);
    inputs.wristManual = manual;
    inputs.wristConnected = wristConnectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void toggleManual() {
    manual = !manual;
  }

  @Override
  public void moveWrist(double speed) {
    wristMotor.set(speed);
  }

  @Override
  public void wristToPosition(double goal, XboxController controller) {
    if (!manual) {
      if (goal > wristEncoder.getPosition()) {
        wristController.setSetpoint(goal, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      } else if (goal < wristEncoder.getPosition()) {
        wristController.setSetpoint(goal, ControlType.kPosition, ClosedLoopSlot.kSlot1);
      }
    } else {
      wristWithJoystick(controller);
    }
  }

  @Override
  public void wristWithJoystick(XboxController controller) {
    moveWrist(controller.getLeftY() / 5);
  }

  @Override
  public void stopWrist() {
    wristMotor.stopMotor();
  }

  @Override
  public void resetWristEncoder() {
    wristEncoder.setPosition(0.0);
  }
}
