package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import com.gemsrobotics.lib.util.MathUtils;
import com.gemsrobotics.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;

public class Intake implements Subsystem {
	private static Intake INSTANCE;

	public static Intake getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Intake();
		}

		return INSTANCE;
	}

	private static final int MOTOR_ID = 10;
	private static final String MOTOR_BUS = Constants.CANBusses.AUX;

	private static final double kP = 0.017 * 1.5;
	private static final double kG = -0.6 / 12.0; // convert to duty cycle

	private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2.0);

	private final TalonFX m_motor;

	public enum Mode {
		POSITION,
		DISABLED
	}

	public enum Position {
		RETRACTED(0),
		BALANCED(9_800),
		MIDDLE(12_000),
		EXTENDED(38_000);

		public static final double TICKS_PER_DEGREE = (EXTENDED.ticks - MIDDLE.ticks) / 90.0;

		public final int ticks;

		Position(final int t) {
			ticks = t;
		}
	}

	private Mode m_mode;
	private double m_reference;

	private Intake() {
		final var motor = MotorControllerFactory.createDefaultTalonFX(MOTOR_ID, MOTOR_BUS);
		motor.setInvertedOutput(false);
		motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);

		m_motor = motor.getInternalController();
		m_motor.config_kP(0, kP);
		m_motor.config_kI(0, 0.0);
		m_motor.config_kD(0, 0.0);
		m_motor.configAllowableClosedloopError(0, Position.TICKS_PER_DEGREE * 5);

		m_mode = Mode.DISABLED;
		m_reference = 0.0;
	}

	public void setDisabled() {
		m_mode = Mode.DISABLED;
	}

	public void setReference(final Position reference) {
		setReference(reference.ticks);
	}

	public void setReference(final double pos) {
		m_mode = Mode.POSITION;
		m_reference = MathUtils.coerce(Position.RETRACTED.ticks, pos, Position.EXTENDED.ticks);
	}

	public Rotation2d getApproximateAngle() {
		final var pos = m_motor.getSelectedSensorPosition();
		return Rotation2d.fromDegrees((pos - Position.MIDDLE.ticks) / Position.TICKS_PER_DEGREE);
	}

	public boolean atReference(final Rotation2d tolerance) {
		final var rawTolerance = tolerance.getDegrees() * Position.TICKS_PER_DEGREE;
		return Math.abs(m_motor.getClosedLoopError()) < rawTolerance;
	}

	public boolean atReference() {
		return atReference(TOLERANCE);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Intake Pos", m_motor.getSelectedSensorPosition());
		switch (m_mode) {
			case POSITION:
				m_motor.set(
						ControlMode.Position, m_reference,
						DemandType.ArbitraryFeedForward, kG * getApproximateAngle().getCos());
				break;
			case DISABLED:
			default:
				m_motor.set(ControlMode.PercentOutput, 0.0);
				break;
		}
	}
}
