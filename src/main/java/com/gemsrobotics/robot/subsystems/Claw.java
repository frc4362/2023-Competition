package com.gemsrobotics.robot.subsystems;

import java.util.Objects;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import com.gemsrobotics.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class Claw implements Subsystem {
	private static final boolean DO_LOGGING = false;

	private static Claw INSTANCE = null;

	private static final int CLAW_DRIVE_MOTOR_ID = 19;
	private static final String CLAW_DRIVE_MOTOR_BUS = Constants.CANBusses.MAIN;
	private static final int CLAW_GRIP_MOTOR_ID = 14;
	private static final String CLAW_GRIP_MOTOR_BUS = Constants.CANBusses.MAIN;

	private static final double TOLERANCE = 0.05;

	private static final double GEARING_MULTIPLIER = 1.0 / 10.5;

	private final MotorController<TalonFX> m_motorDrive;
	private final MotorController<TalonFX> m_motorGrip;

	private final PIDController m_controllerGrip;

	//Proportion of claw to close: 1.00 is closed 0.0 is fully open against hard stop
	private double m_reference;

	public static Claw getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Claw();
		}

		return INSTANCE;
	}

	public Claw() {
		m_motorDrive = MotorControllerFactory.createDefaultTalonFX(CLAW_DRIVE_MOTOR_ID, CLAW_DRIVE_MOTOR_BUS);
		m_motorDrive.setInvertedOutput(true);
		m_motorDrive.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);

		m_motorGrip = MotorControllerFactory.createDefaultTalonFX(CLAW_GRIP_MOTOR_ID, CLAW_GRIP_MOTOR_BUS);
		m_motorGrip.setInvertedOutput(true);
		m_motorGrip.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motorGrip.setGearingParameters(GEARING_MULTIPLIER, 1.0);
		
		m_controllerGrip = new PIDController(0.0, 0.0, 0.0); //TODO

		m_reference = 0.0;//
	}

	public enum State {
		NEUTRAL,
		GRIPPING,
		OPEN
	}

	public void setReference(final double reference) {
		//sanitize input
		if (reference > 0.0 || reference < 1.0) {
			//throw log error
			return;
		}

		m_reference = reference;
	}

	public double getPosition() {
		return 0;//m_grip_motor.getPositionRotations() and then do math to proportion;
	}

	 public boolean atReference(final double toleranceRotations) {
	 	return Math.abs(m_reference - m_motorDrive.getPositionRotations()) < toleranceRotations;
	 }

	 public boolean atReference() {
		return atReference(TOLERANCE);
	 }

	@Override
	public void periodic() {
		m_motorGrip.setVoltage(m_controllerGrip.calculate(m_motorGrip.getPositionRotations(), m_reference));
	}
}
