package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import com.gemsrobotics.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;
import static java.lang.Math.abs;

public class Pivot implements Subsystem {
	private static final boolean DO_LOGGING = false;

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

	// at 0 position units, arm 54 degrees off of the horizontal
	private static final Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(90); // 54 degrees is the starting angle

	// limits calculated from stowed starting position (54 degrees)
	private static final int
			FORWARD_SOFT_LIMIT = 60_000,
			REVERSE_SOFT_LIMIT = -365_000;

	private static final double
			kP = 252,
			kD = 4;
	private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(0.5);


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
		m_feedforward = new ArmFeedforward(0.103, 0.27, 0.0, 0.0);

		setReference(STARTING_ANGLE);
	}

	public void setReference(final Rotation2d reference) {
		m_reference = reference;
	}

	public Rotation2d getPosition() {
		return Rotation2d.fromRotations(m_motor.getPositionRotations()).rotateBy(STARTING_ANGLE);
	}

	public boolean atReference(final Rotation2d tolerance) {
		return abs(m_controllerPivot.getPositionError()) < tolerance.getRadians();
	}

	public boolean atReference() {
		return atReference(TOLERANCE);
	}

	@Override
	public void periodic() {
		final var ff = m_feedforward.calculate(getPosition().getRadians(), 0.0);
		m_motor.setVoltage(m_controllerPivot.calculate(getPosition().getRadians(), m_reference.getRadians()), ff);

		if (DO_LOGGING) {
			SmartDashboard.putNumber("Pivot Angle", getPosition().getDegrees());
			SmartDashboard.putNumber("Pivot Drawn Amps", m_motor.getDrawnCurrentAmps());
			SmartDashboard.putNumber("Pivot Control Effort", m_motor.getVoltageOutput());
			SmartDashboard.putNumber("Pivot Velocity", m_motor.getVelocityAngularRadiansPerSecond());
		}
	}
}
