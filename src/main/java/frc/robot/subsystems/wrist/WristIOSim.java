// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.wrist.WristConstants.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/** Add your docs here. */
public class WristIOSim implements WristIO{
	private final DCMotor wristDCMotor;
	private final DCMotorSim wristMotorSim;

	public WristIOSim(DCMotor motor, double moi, double gearRatio) {
		wristDCMotor = 
			new DCMotor(
				12.0,
				3.6,
				211,
				3.6,
				RPM.of(6784).in(RadiansPerSecond),
				1
			);

		wristMotorSim =
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(wristDCMotor, wristMotorMOI, wristGearRatio),
				wristDCMotor
			);
	}
}
