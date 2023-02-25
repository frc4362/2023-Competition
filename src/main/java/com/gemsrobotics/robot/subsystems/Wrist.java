package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import com.gemsrobotics.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;

import static java.lang.Math.abs;

public final class Wrist implements Subsystem {
	private static final int MOTOR_ID = 12;
	private static final String MOTOR_BUS = Constants.CANBusses.MAIN;

	private static Wrist INSTANCE;

	public static Wrist getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Wrist();
		}

		return INSTANCE;
	}

	private final MotorController<TalonFX> m_motor;
	private final PIDController m_controller;

	private Rotation2d m_reference;

	private Wrist() {
		m_motor = MotorControllerFactory.createDefaultTalonFX(MOTOR_ID, MOTOR_BUS);
		m_motor.setInvertedOutput(true);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);

		m_controller = new PIDController(0.0, 0.0, 0.0);

		m_reference = Rotation2d.fromDegrees(0.0);
	}

	public void setReference(final Rotation2d reference) {
		m_reference = reference;
	}

	public Rotation2d getPosition() {
		return Rotation2d.fromRotations(m_motor.getPositionRotations());//.rotateBy(STARTING_ANGLE);
	}

	public boolean atReference(final Rotation2d tolerance) {
		return false;//abs(m_controllerPivot.getPositionError()) < tolerance.getRadians();
	}

//	public boolean atReference() {
//		return atReference(TOLERANCE);
//	}

	@Override
	public void periodic() {
		m_motor.setVoltage(m_controller.calculate(m_motor.getPositionRotations()));
	}
}
