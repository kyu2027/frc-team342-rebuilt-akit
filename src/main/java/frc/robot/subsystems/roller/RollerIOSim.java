// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.roller.RollerConstants.rollerGearRatio;
import static frc.robot.subsystems.roller.RollerConstants.rollerMotorMOI;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.LoggedRobot;

/** Add your docs here. */
public class RollerIOSim implements RollerIO {
  private final DCMotor rollerDCMotor;
  private final DCMotorSim rollerMotorSim;

  public RollerIOSim() {
    rollerDCMotor = new DCMotor(
      12.0,
      7.09,
      366.0,
      2.0,
      RPM.of(6000).in(RadiansPerSecond),
      1);

    rollerMotorSim =
      new DCMotorSim(
        LinearSystemId.createDCMotorSystem(rollerDCMotor, rollerMotorMOI, rollerGearRatio),
        rollerDCMotor);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    if(DriverStation.isDisabled()) {
      stopRoller();
    }else{
      setVoltage(inputs.rollerAppliedVoltage);
    }

    inputs.rollerConnected = true;
    rollerMotorSim.update(LoggedRobot.defaultPeriodSecs);

    inputs.rollerVelocity = RPM.of(rollerMotorSim.getAngularVelocityRPM());

    inputs.rollerAppliedVoltage = rollerMotorSim.getInputVoltage();
    inputs.rollerSupplyCurrentAmps = rollerMotorSim.getCurrentDrawAmps();
  }

  @Override
  public void spinRoller(double speed) {
    rollerMotorSim.setInputVoltage(speed);
  }

  @Override
  public void setVoltage(double voltage) {
    double volts = MathUtil.clamp(voltage, -12, 12);
    rollerMotorSim.setInputVoltage(volts);
  }

  @Override
  public void stopRoller() {
    rollerMotorSim.setInputVoltage(0.0);
  }
}
