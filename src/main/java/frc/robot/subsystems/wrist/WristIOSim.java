// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.wrist.WristConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.LoggedRobot;

/** Add your docs here. */
public class WristIOSim implements WristIO {
  private final DCMotor wristDCMotor;
  private final DCMotorSim wristMotorSim;

  private final ProfiledPIDController wristPID;
  private final ArmFeedforward wristFF;

  private double goal;

  public WristIOSim(DCMotor motor, double moi, double gearRatio) {
    wristDCMotor = new DCMotor(12.0, 3.6, 211, 3.6, RPM.of(6784).in(RadiansPerSecond), 1);

    wristMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(wristDCMotor, wristMotorMOI, wristGearRatio),
            wristDCMotor);

    goal = 0.0;

    wristPID =
        new ProfiledPIDController(
            wristProfiledPIDValues[0],
            wristProfiledPIDValues[1],
            wristProfiledPIDValues[2],
            wristProfiledPIDConstraints);

    wristFF =
        new ArmFeedforward(
            wristFeedforwardValues[0],
            wristFeedforwardValues[1],
            wristFeedforwardValues[2],
            wristFeedforwardValues[3]);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stopWrist();
    } else {
      wristToPosition(wristMotorSim.getInputVoltage());
    }

    inputs.wristConnected = true;
    wristMotorSim.update(LoggedRobot.defaultPeriodSecs);

    inputs.wristVelocity = wristMotorSim.getAngularVelocityRadPerSec();
    inputs.wristPosition = wristMotorSim.getAngularPositionRad();
    inputs.wristGoal = this.goal;

    inputs.wristAppliedVoltage = wristMotorSim.getInputVoltage();
    inputs.wristSupplyCurrentAmps = wristMotorSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double voltage) {
    wristMotorSim.setInputVoltage(voltage);
  }

  @Override
  public void wristToPosition(double goal) {
    this.goal = MathUtil.clamp(goal, wristStartingPosition, wristExtendedPosition);

    double pidOutput = wristPID.calculate(wristMotorSim.getAngularPositionRad(), goal);
    double feedforward =
        wristFF.calculate(
            wristMotorSim.getAngularPositionRad(), wristMotorSim.getAngularVelocityRadPerSec());
    double volts = pidOutput + feedforward;
    double safeVolts = MathUtil.clamp(volts, -12, 12);

    wristMotorSim.setInputVoltage(safeVolts);
  }

  @Override
  public void stopWrist() {
    wristMotorSim.setInputVoltage(0.0);
  }

  @Override
  public void resetWristEncoder() {
    wristMotorSim.setAngle(0.0);
  }
}
