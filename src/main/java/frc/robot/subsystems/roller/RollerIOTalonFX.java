// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import static frc.robot.subsystems.roller.RollerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class RollerIOTalonFX implements RollerIO {
  private final TalonFX rollerMotor;

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrentAmps;

  public RollerIOTalonFX() {
    rollerMotor = new TalonFX(rollerMotorID);

    velocity = rollerMotor.getVelocity();
    appliedVoltage = rollerMotor.getMotorVoltage();
    supplyCurrentAmps = rollerMotor.getSupplyCurrent();

    var rollerConfig = new TalonFXConfiguration();
    rollerConfig
        .CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(rollerCurrentLimit);
    rollerConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    rollerMotor.getConfigurator().apply(rollerConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, velocity, appliedVoltage, supplyCurrentAmps);

    ParentDevice.optimizeBusUtilizationForAll(rollerMotor);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.rollerConnected =
        BaseStatusSignal.refreshAll(velocity, appliedVoltage, supplyCurrentAmps).isOK();

    inputs.rollerVelocity = velocity.getValue();

    inputs.rollerAppliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
  }

  @Override
  public void spinRoller(double speed) {
    rollerMotor.set(speed);
  }

  @Override
  public void stopRoller() {
    rollerMotor.stopMotor();
  }
}
