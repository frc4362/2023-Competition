package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MyClaw implements Subsystem {
	private final MotorController<TalonFX> m_motor;

	public MyClaw() {
		m_motor = MotorControllerFactory.createDefaultTalonFX(14, "rio");
		m_motor.setInvertedOutput(false);
	}

	public enum State {
		NEUTRAL,
		GRIPPING,
		OPEN
	}

	@Override
	public void periodic() {

	}
}
