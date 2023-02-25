package com.gemsrobotics.robot.subsystems;

import java.util.Objects;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import com.gemsrobotics.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MyClaw implements Subsystem {

	private static final boolean DO_LOGGING = false;

	private static MyClaw INSTANCE = null;

	private static final int CLAW_DRIVE_MOTOR_ID = 19;
	private static final String CLAW_DRIVE_MOTOR_BUS = Constants.CANBusses.MAIN;
	private static final int CLAW_GRIP_MOTOR_ID = 14;
	private static final String CLAW_GRIP_MOTOR_BUS = Constants.CANBusses.MAIN;

	private final MotorController<TalonFX> m_drive_motor;
	private final MotorController<TalonFX> m_grip_motor;

	private final PIDController m_grip_controller;

	//Proportion of claw to close: 1.00 is closed 0.0 is fully open against hard stop
	private double m_reference;

	public static MyClaw getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new MyClaw();
		}
		return INSTANCE;
	}

	public MyClaw() {
		m_drive_motor = MotorControllerFactory.createDefaultTalonFX(CLAW_DRIVE_MOTOR_ID, CLAW_DRIVE_MOTOR_BUS);
		m_drive_motor.setInvertedOutput(true);
		m_drive_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);

		m_grip_motor = MotorControllerFactory.createDefaultTalonFX(CLAW_GRIP_MOTOR_ID, CLAW_GRIP_MOTOR_BUS);
		m_grip_motor.setInvertedOutput(true);
		m_grip_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		
		m_grip_controller = new PIDController(0.0, 0.0, 0.0); //TODO

		m_reference = 0.0;//
	}

	public enum State {
		NEUTRAL,
		GRIPPING,
		OPEN
	}

	public void setReference(final double reference) {
		//sanitize input
		if(reference > 0.0 || reference < 1.0) {
			//throw log error
			return;
		}
		m_reference = reference;
	}

	public double getPosition() {
		return //m_grip_motor.getPositionRotations() and then do math to proportion;
	}

	// public boolean atReference(final double tolerance) {
	// 	return false;//abs(m_controllerPivot.getPositionError()) < tolerance.getRadians();
	// }

	@Override
	public void periodic() {
		m_grip_motor.setVoltage(m_grip_controller.calculate(m_grip_motor.getPositionRotations()));
		//m_drive_motor. do a thing
	}
}
