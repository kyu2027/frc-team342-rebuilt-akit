// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static frc.robot.subsystems.wrist.WristConstants.*;
import static frc.robot.util.SparkUtil.*;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/** Add your docs here. */
public class WristIOSpark implements WristIO {
  private final SparkFlex wristMotor;
  private final RelativeEncoder wristEncoder;

  private final ProfiledPIDController wristController;
  private final ArmFeedforward wristFeedforward;

  private final Debouncer wristConnectedDebouncer;

  private boolean manual;
  private double goal;

  public WristIOSpark() {
    wristMotor = new SparkFlex(wristMotorID, MotorType.kBrushless);
    wristEncoder = wristMotor.getEncoder();

    wristController = new ProfiledPIDController(
      wristProfiledPIDValues[0], 
      wristProfiledPIDValues[1], 
      wristProfiledPIDValues[2], 
      wristProfiledPIDConstraints);

    wristFeedforward = new ArmFeedforward(
      wristFeedforwardValues[0], 
      wristFeedforwardValues[1], 
      wristFeedforwardValues[2],
      wristFeedforwardValues[3]);

    wristConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

    manual = false;

    var wristConfig = new SparkFlexConfig();
    wristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(wristCurrentLimit);
    wristConfig.encoder.positionConversionFactor(wristPositionConversionFactor);

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
    ifOk(wristMotor, () -> goal, (value) -> inputs.wristGoal = value);
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
  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }

  @Override
  public void wristToPosition(double goal, XboxController controller) {
    this.goal = goal;

    if (!manual) {
      double pidOutput = wristController.calculate(wristEncoder.getPosition(), goal);
      double feedforward = wristFeedforward.calculate(wristEncoder.getPosition(), wristEncoder.getVelocity());
      double volts = pidOutput + feedforward;
      double safeVolts = MathUtil.clamp(volts, -12, 12);

      wristMotor.setVoltage(safeVolts);
    } else {
      wristWithJoystick(controller);
    }
  }

  @Override
  public void wristWithJoystick(XboxController controller) {
    wristMotor.set(controller.getLeftY() / 5);
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
