package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import com.gemsrobotics.lib.util.MathUtils;
import com.gemsrobotics.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;

import static java.lang.Math.signum;

public final class Wrist implements Subsystem {
	private static Wrist INSTANCE;

	public static Wrist getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Wrist();
		}

		return INSTANCE;
	}

	private static final int MOTOR_ID = 12;
	private static final String MOTOR_BUS = Constants.CANBusses.MAIN;

	private static final double kG = 0.3;
	private static final double kS = 0.07;

	private static final double GEARING_MULTIPLIER = 1 / 38.94;
	private static final double TOLERANCE = 1.25;
	public static final Rotation2d STARTING_ANGLE_FROM_ELEVATOR = Rotation2d.fromDegrees(230);//-54
	public static final int MAX_FEEDBACK_VOLTS = 4;

	private final MotorController<TalonFX> m_motor;
	private final ArmFeedforward m_feedforward;
	private final PIDController m_controller;

	private Rotation2d m_externalAngle;
	// rotations
	private double m_referenceRotations;

	private Wrist() {
		m_motor = MotorControllerFactory.createDefaultTalonFX(MOTOR_ID, MOTOR_BUS);
		m_motor.setInvertedOutput(false);
		m_motor.setOpenLoopVoltageRampRate(0.25);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motor.setGearingParameters(GEARING_MULTIPLIER, 1.0);
		m_motor.getInternalController().configNeutralDeadband(0.04);

		// m_motor.getInternalController().configNominalOutputForward(GEARING_MULTIPLIER);

		m_controller = new PIDController(18.0, 0.0, 0.0);
		m_controller.setTolerance(0.0);

		m_feedforward = new ArmFeedforward(kS, kG, 0.0, 0.0);

		m_externalAngle = Rotation2d.fromDegrees(0.0);
		m_referenceRotations = 0.638;//STARTING_ANGLE_FROM_ELEVATOR.getRotations();
	}

	// This is in angle relative to the elevator
	// Negative numbers are forwards from the elevator
	public enum Position {
		STARTING(Rotation2d.fromDegrees(230)), //-54
		SCORING_MID(Rotation2d.fromDegrees(-16)),
		SCORING_HIGH(Rotation2d.fromDegrees(-16)),
		HAT(Rotation2d.fromDegrees(-41)),
		SHELF_PICKUP(Rotation2d.fromDegrees(-50)),
		CLEAR(Rotation2d.fromDegrees(0)),
		STOWED(Rotation2d.fromDegrees(210));

		public final Rotation2d rotation;

		Position(final Rotation2d rot) {
			rotation = rot;
		}
	}

	public void setReferencePosition(final Position referencePosition) {
		setReferenceRotationsToElevator(referencePosition.rotation.getRotations());
	}

	public void setReferenceRotationsToElevator(final double referenceRotations) {
		m_referenceRotations = referenceRotations;
	}

	// we need these two methods so that we can cross the 180 degree boundary; they do different things
	public double getRotationsToElevator() {
		return m_motor.getPositionRotations() + STARTING_ANGLE_FROM_ELEVATOR.getRotations();
	}

	public Rotation2d getAngleToElevator() {
		return Rotation2d.fromRotations(m_motor.getPositionRotations()).rotateBy(STARTING_ANGLE_FROM_ELEVATOR);
	}

	public Rotation2d getAngleToGround() {
		return getAngleToElevator().plus(m_externalAngle);
	}

	public boolean atReference(final double tolerance) {
		return Math.abs(m_referenceRotations - m_motor.getPositionRotations()) < tolerance;
	}

	public boolean atReference() {
		return atReference(TOLERANCE);
	}

	public void setExternalAngle(final Rotation2d externalAngle) {
		m_externalAngle = externalAngle;
	}

	public boolean isSafeFromElevatorCollision() {
		return getAngleToElevator().getDegrees() > -37 && getAngleToElevator().getDegrees() < 0;
	}

	public void log() {
		// SmartDashboard.putNumber("Wrist Volts", m_motor.getVoltageOutput());
		// SmartDashboard.putNumber("Wrist Feedforward", m_feedforward.calculate(getAngleToGround().getRadians(), 0.0));
		// SmartDashboard.putNumber("Wrist External Angle", m_externalAngle.getDegrees());
	
		SmartDashboard.putNumber("Wrist Position Degrees", getRotationsToElevator() * 360);
		SmartDashboard.putNumber("Wrist Reference Degrees", m_referenceRotations * 360);
		SmartDashboard.putNumber("Wrist AtRef", atReference() ? 1.0 : 0.0);
	}

	@Override
	public void periodic() {
		final var ff = m_feedforward.calculate(getAngleToGround().getRadians(), 0.0);
		var feedback = m_controller.calculate(getRotationsToElevator(), m_referenceRotations);
		final var clampedFeedback = MathUtils.coerce(-MAX_FEEDBACK_VOLTS, feedback, MAX_FEEDBACK_VOLTS);

		m_motor.setVoltage(clampedFeedback, ff + kS * signum(feedback));
	}
}
