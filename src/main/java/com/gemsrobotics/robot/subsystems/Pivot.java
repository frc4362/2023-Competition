package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import com.gemsrobotics.lib.util.MathUtils;
import com.gemsrobotics.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;
import static java.lang.Math.abs;

public class Pivot implements Subsystem {
	private static Pivot INSTANCE = null;

	public static Pivot getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Pivot();
		}

		return INSTANCE;
	}

	private static final int MOTOR_ID = 15;
	private static final String MOTOR_BUS = Constants.CANBusses.AUX;
	public static final double GEARING_MULTIPLIER = 1.0 / 531.67;

	// limits calculated from stowed starting position (54 degrees)
	private static final int
			FORWARD_SOFT_LIMIT = 60_000,
			REVERSE_SOFT_LIMIT = -365_000;

	private static final double
			kP = 126 / 2,//126,
			kD = 2;

	private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2.0);

	private final MotorController<TalonFX> m_motor;
	private final PIDController m_controllerPivot;
	private final ArmFeedforward m_feedforward;

	private Rotation2d m_reference;

	private Pivot() {
		m_motor = MotorControllerFactory.createDefaultTalonFX(MOTOR_ID, MOTOR_BUS);
		m_motor.setInvertedOutput(false);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motor.setGearingParameters(GEARING_MULTIPLIER, 1.0);

		m_motor.getInternalController().configForwardSoftLimitEnable(true);
		m_motor.getInternalController().configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT);
		m_motor.getInternalController().configReverseSoftLimitEnable(true);
		m_motor.getInternalController().configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT);

		m_controllerPivot = new PIDController(kP, 0.0, kD);
		m_controllerPivot.setTolerance(TOLERANCE.getRadians());
		m_feedforward = new ArmFeedforward(0.103, 0.27, 9.2, 0.136);

		setReference(Position.STARTING.rotation);
	}

	// Angles are positive coming up out of the ground
	public enum Position {
		STARTING(					Rotation2d.fromDegrees(54)),
		STOWED(						Rotation2d.fromDegrees(54)),
		RETURNED(					Rotation2d.fromDegrees(65)),
		SHELF_PICKUP(				Rotation2d.fromDegrees(54)),
		SCORING(					Rotation2d.fromDegrees(40.5)),
		AUTON_SCORING(				Rotation2d.fromDegrees(39.5)),
		AUTON_SCORING_BLUE_STATES(	Rotation2d.fromDegrees(39.5)),
		HATTING_HIGH(				Rotation2d.fromDegrees(33.5)),
		HATTING_MID(				Rotation2d.fromDegrees(30.5));

		public final Rotation2d rotation;

		Position(final Rotation2d rot) {
			rotation = rot;
		}
	}

	public void setReference(final Rotation2d reference) {
		m_reference = reference;
	}

	public void setReference(final Position setpoint) {
		setReference(setpoint.rotation);
	}

	public Rotation2d getAngle() {
		return Rotation2d.fromRotations(m_motor.getPositionRotations()).rotateBy(Position.STARTING.rotation);
	}

	public boolean atReference(final Rotation2d tolerance) {
		return abs(getErrorDegrees()) < tolerance.getDegrees();
	}

	public boolean atReference() {
		return atReference(TOLERANCE);
	}

	public double getErrorDegrees() {
		return m_reference.getDegrees() - getAngle().getDegrees();
	}

	public boolean isInhibitingMobility() {
		return getAngle().getDegrees() < 50;
	}

	public void log() {
		// SmartDashboard.putNumber("Pivot Angle", getAngle().getDegrees());
		// SmartDashboard.putNumber("Pivot Drawn Amps", m_motor.getDrawnCurrentAmps());
		// SmartDashboard.putNumber("Pivot Control Effort", m_motor.getVoltageOutput());
		// SmartDashboard.putNumber("Pivot Velocity", m_motor.getVelocityAngularRadiansPerSecond());
		// SmartDashboard.putNumber("Pivot Error", getErrorDegrees());
		SmartDashboard.putNumber("Pivot AtRef", atReference() ? 1.0 : 0.0);
	}

	public void scrubSetpoint(final double rateDegrees) {
		//Note this plus limits the range of our output to +/-PI
		//m_reference = m_reference.plus(Rotation2d.fromDegrees(rateDegrees));
		m_reference = Rotation2d.fromDegrees(m_reference.getDegrees()+rateDegrees);
	}

	@Override
	public void periodic() {
		final var ff = m_feedforward.calculate(getAngle().getRadians(), 0.0);
		final var feedback = m_controllerPivot.calculate(getAngle().getRadians(), m_reference.getRadians());
		m_motor.setVoltage(MathUtils.coerce(-12, feedback, 12), ff);
	}
}
