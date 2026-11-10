// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.wrist.WristConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class WristIOSim implements WristIO {
  private final DCMotor wristDCMotor;
  private final DCMotorSim wristMotorSim;

  private final ProfiledPIDController wristPID;
  private final ArmFeedforward wristFF;

  private boolean manual;
  private double goal;

  public WristIOSim(DCMotor motor, double moi, double gearRatio) {
    wristDCMotor = new DCMotor(
		  12.0,
		  3.6,
		  211,
		  3.6,
		  RPM.of(6784).in(RadiansPerSecond),
		  1);

    wristMotorSim =
      new DCMotorSim(
        LinearSystemId.createDCMotorSystem(wristDCMotor, wristMotorMOI, wristGearRatio),
        wristDCMotor);

    manual = false;
    goal = 0.0;

    wristPID = new ProfiledPIDController(
      wristProfiledPIDValues[0],
      wristProfiledPIDValues[1],
      wristProfiledPIDValues[2],
      wristProfiledPIDConstraints);

    wristFF = new ArmFeedforward(
      wristFeedforwardValues[0],
      wristFeedforwardValues[1],
      wristFeedforwardValues[2],
      wristFeedforwardValues[3]
    );
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
	  if(DriverStation.isDisabled()) {
		  stopWrist();
	  }else{
      
    }
  }

  @Override
  public void setVoltage(double voltage) {
    wristMotorSim.setInputVoltage(voltage);
  }

  @Override
  public void toggleManual() {
    manual = !manual;
  }

  @Override
  public void wristToPosition(double goal, XboxController controller) {
    if(manual) {
      
    }
  }

  @Override
  public void wristWithJoystick(XboxController controller) {}
}
