package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import com.gemsrobotics.lib.drivers.MotorController.NeutralBehaviour;
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

	private static final int POSITION_MOTOR_ID = 10;
	private static final String POSITION_MOTOR_BUS = Constants.CANBusses.AUX;

	private static final int DRIVE_MOTOR_ID = 20;
	private static final String DRIVE_MOTOR_BUS = Constants.CANBusses.MAIN;

	private static final double kP = 0.017 * 1.5;
	private static final double kG = -0.9 / 12.0; // convert to duty cycle

	private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(3.5);

	private final TalonFX m_motorPosition;
	private final TalonFX m_motorDrive;
	private final TalonFX m_motorExhaust;

	public enum Mode {
		POSITION,
		DISABLED
	}

	public enum State {
		RETRACTED(0, 0.0, 0),
		BALANCED(9_800, 0.0, 0),
		MIDDLE(14_000, 0.0, 0), // 25 000
		EXTENDED(38_000, 0.0, 0),
		INTAKING(38_000, 0.4, 0.4), //e 0.05
		OUTTAKING(0, -1, 1.0);

		public static final double TICKS_PER_DEGREE = (EXTENDED.ticks - 12_000) / 90.0;

		public final int ticks;
		public final double drive;
		public final double exhaust;

		State(final int t, final double d, final double e) {
			ticks = t;
			drive = d;
			exhaust = e;
		}
	}

	private Mode m_mode;
	private State m_state;

	private Intake() {
		final var positionMotor = MotorControllerFactory.createDefaultTalonFX(POSITION_MOTOR_ID, POSITION_MOTOR_BUS);
		positionMotor.setInvertedOutput(false);
		positionMotor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);

		m_motorPosition = positionMotor.getInternalController();
		m_motorPosition.config_kP(0, kP);
		m_motorPosition.config_kI(0, 0.0);
		m_motorPosition.config_kD(0, 0.0);
		m_motorPosition.configAllowableClosedloopError(0, State.TICKS_PER_DEGREE * 1.5);
		m_motorPosition.configNeutralDeadband(0.02);

		m_motorPosition.configNominalOutputReverse(-0.04);
		m_motorPosition.configNominalOutputForward(0.04);

		m_motorPosition.configForwardSoftLimitThreshold(State.EXTENDED.ticks);
		m_motorPosition.configReverseSoftLimitThreshold(State.RETRACTED.ticks);
		m_motorPosition.overrideSoftLimitsEnable(true);

//		m_motorPosition.configAllowableClosedloopError(0, State.TICKS_PER_DEGREE * 2.5);

		final var driveMotor = MotorControllerFactory.createDefaultTalonFX(DRIVE_MOTOR_ID, DRIVE_MOTOR_BUS);
		driveMotor.setInvertedOutput(true);
		driveMotor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);

		m_motorDrive = driveMotor.getInternalController();
		m_motorDrive.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
				true,
				100,
				100,
				100));

		final var exhaust = MotorControllerFactory.createDefaultTalonFX(16, "rio");
		exhaust.setInvertedOutput(false);
		exhaust.setNeutralBehaviour(NeutralBehaviour.BRAKE);

		m_motorExhaust = exhaust.getInternalController();

		m_mode = Mode.DISABLED;
		m_state = State.RETRACTED;
	}

	public boolean isExhaustStalled() {
		return m_motorExhaust.getStatorCurrent() > 60; //20
	}

	public void setDisabled() {
		m_mode = Mode.DISABLED;
	}

	public void setState(final State state) {
		m_mode = Mode.POSITION;
		m_state = state;
	}

	public Rotation2d getApproximateAngle() {
		final var pos = m_motorPosition.getSelectedSensorPosition();
		return Rotation2d.fromDegrees((pos - State.MIDDLE.ticks) / State.TICKS_PER_DEGREE);
	}

	private double getError() {
		return m_motorPosition.getSelectedSensorPosition() - m_state.ticks;
	}

	public boolean atReference(final Rotation2d tolerance) {
		final var rawTolerance = tolerance.getDegrees() * State.TICKS_PER_DEGREE;
		return Math.abs(getError()) < rawTolerance;
	}

	public boolean atReference() {
		return Math.abs(m_motorPosition.getClosedLoopError()) < State.TICKS_PER_DEGREE * 5;
		// return atReference(TOLERANCE);
	}

	public void log() {
		SmartDashboard.putNumber("Intake Amps Drawn", m_motorDrive.getStatorCurrent());
		SmartDashboard.putNumber("Intake Pos", m_motorPosition.getSelectedSensorPosition());
		SmartDashboard.putNumber("Intake Error", getError());
		SmartDashboard.putNumber("Exhaust Current", m_motorExhaust.getStatorCurrent());
	}

	@Override
	public void periodic() {
		switch (m_mode) {
			case POSITION:
				m_motorPosition.set(
						ControlMode.Position, m_state.ticks,
						DemandType.ArbitraryFeedForward, kG * getApproximateAngle().getCos());
				m_motorDrive.set(ControlMode.PercentOutput, m_state.drive);
				m_motorExhaust.set(ControlMode.PercentOutput, m_state.exhaust);
				break;
			case DISABLED:
			default:
				m_motorPosition.set(ControlMode.PercentOutput, 0.0);
				m_motorDrive.set(ControlMode.PercentOutput, 0.0);
				m_motorExhaust.set(ControlMode.PercentOutput, 0.0);
				break;
		}
	}
}
