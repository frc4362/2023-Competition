package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import com.gemsrobotics.lib.util.MathUtils;
import com.gemsrobotics.lib.util.Units;

import com.gemsrobotics.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

public final class Elevator implements Subsystem {
	private static Elevator INSTANCE = null;

	public static Elevator getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Elevator();
		}

		return INSTANCE;
	}

	private static final int MOTOR_ID = 18;
	private static final String MOTOR_BUS = Constants.CANBusses.MAIN;

	// limits calculated from starting position at the bottom
	private static final double
			FORWARD_SOFT_LIMIT = 125_000,
			REVERSE_SOFT_LIMIT = 0;
	private static final double TOLERANCE_METERS = 0.05;
	public static final double GEARING_MULTIPLIER = 1.0 / 9.0;

	public static final double
		kS = 0.15,
		kG = 0.66;

	private final MotorController<TalonFX> m_motor;
	private final PIDController m_controller;

	private double m_referenceMeters;
	private Rotation2d m_externalAngle;

	private Elevator() {
		m_motor = MotorControllerFactory.createDefaultTalonFX(MOTOR_ID, MOTOR_BUS);
		m_motor.setInvertedOutput(false);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		// TODO
		m_motor.setOpenLoopVoltageRampRate(0.2);

		m_motor.getInternalController().configNeutralDeadband(0.01);
		m_motor.getInternalController().configPeakOutputForward(1.0);
		m_motor.getInternalController().configPeakOutputReverse(-1.0);

//		m_motor.getInternalController().configNominalOutputForward(kS);
//		m_motor.getInternalController().configNominalOutputReverse(-kS);

		m_motor.getInternalController().configForwardSoftLimitEnable(true);
		m_motor.getInternalController().configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT);
		m_motor.getInternalController().configReverseSoftLimitEnable(true);
		m_motor.getInternalController().configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT);
		m_motor.getInternalController().overrideSoftLimitsEnable(true);

		m_motor.setGearingParameters(GEARING_MULTIPLIER, Units.inches2Meters(4.0*2)/(2.0*Math.PI));

		// 142.7
		m_controller = new PIDController(12.2, 0.0, 0.0);

		m_referenceMeters = Position.SAFETY_BOTTOM.extensionMeters;
		m_externalAngle = Rotation2d.fromDegrees(90);
	}

	public enum Position {
		TRUE_BOTTOM(0.0),
		SAFETY_BOTTOM(0.005),
		FRONT_SAFETY(0.5),
		SCORING_MID(0.7),
		SAFETY_TOP(1.38),
		TRUE_TOP(1.38); // meters

		public final double extensionMeters;

		Position(final double p) {
			extensionMeters = p;
		}
	}

	public void setReference(final Position pos) {
		setReference(pos.extensionMeters);
	}

	public void setReference(final double referenceMeters) {
		m_referenceMeters = MathUtils.coerce(
				Position.SAFETY_BOTTOM.extensionMeters,
				referenceMeters,
				Position.SAFETY_TOP.extensionMeters);
	}

	public void setOpenLoop(final double openLoopDemand) {
		m_motor.setDutyCycle(openLoopDemand);
	}

	public void setExternalAngle(final Rotation2d angle) {
		m_externalAngle = angle;
	}

	public boolean atReference(final double tolerance) {
		return abs(m_referenceMeters - m_motor.getPositionMeters()) < tolerance;
	}

	public boolean atReference() {
		return atReference(TOLERANCE_METERS);
	}

	public boolean isAbove(final Position position) {
		return m_motor.getPositionMeters() >= position.extensionMeters;
	}

	public boolean isBelow(final Position position) {
		return m_motor.getPositionMeters() <= position.extensionMeters;
	}

	public double getHeightMeters() {
		return m_motor.getPositionMeters();
	}

	public void log() {
		SmartDashboard.putNumber("Elevator Extension", getHeightMeters());
		SmartDashboard.putNumber("Elevator Angle", m_externalAngle.getDegrees());
		SmartDashboard.putNumber("Elevator Control Effort", m_motor.getVoltageOutput());
		SmartDashboard.putNumber("Elevator Raw Measurement", m_motor.getInternalController().getSelectedSensorPosition());
	}

	@Override
	public void periodic() {
		final var error = abs(getHeightMeters() - m_referenceMeters);
		final var feedback = m_controller.calculate(getHeightMeters(), m_referenceMeters);
		final var limitedFeedback = MathUtils.coerce(-7, feedback, 7);
		m_motor.setVoltage(limitedFeedback, kG * m_externalAngle.getSin() + kS * signum(limitedFeedback));
	}
}
