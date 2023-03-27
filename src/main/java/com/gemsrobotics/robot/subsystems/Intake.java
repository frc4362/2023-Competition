package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import com.gemsrobotics.lib.drivers.MotorController.NeutralBehaviour;
import com.gemsrobotics.robot.Constants;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;
import java.util.Optional;

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
	private static final double kP = 0.028; // 0.018
	private static final double kG = -0.45 / 12.0; // convert to duty cycle

	private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(4.0);

	private final TalonFX m_motorPosition;
	private final TalonFX m_rollerBottom;
	private final TalonFX m_rollerTop;
	private final DigitalInput m_beamBreak;

	public enum Mode {
		POSITION,
		DISABLED
	}

	public enum State {
		RETRACTED(0, 0.0, 0, false),
		BALANCED(9_800, 0.0, 0, false),
		MIDDLE(11_000, 0.0, 0, false), // 25 000
		EXTENDED(29_850, 0.0, 0, false),
		INTAKING(29_850, -0.5, .5, false), //e 0.05
		OUTTAKING_HIGH(0, .775, -0.6, false),
		OUTTAKING_HIGH_AUTO(0, 0.9, -0.8, false),
		OUTTAKING_MID(0, 0.4, -0.4, true),
		OUTTAKING_HYBRID(0, 0.25,-0.25, true),
		OUTTAKING_BOWLING(12_000, 1.0, -1.0, true),
		CLEAR_INTAKE(25_000, 0.5, -0.5, true),
		CENTER(0, -1.0, 1.0, false);

		public static final double TICKS_PER_DEGREE = (EXTENDED.ticks - 4_700) / 90.0;

		public final int ticks;
		public final double topDutyCycle;
		public final double bottomDutyCycle;
		public final boolean isOuttake;

		State(final int t, final double d, final double e, final boolean o) {
			ticks = t;
			topDutyCycle = d;
			bottomDutyCycle = e;
			isOuttake = o;
		}
	}

	public enum TargetHeight {
		HIGH(State.OUTTAKING_HIGH),
		HIGH_AUTO(State.OUTTAKING_HIGH_AUTO),
		MID(State.OUTTAKING_MID),
		HYBRID(State.OUTTAKING_HYBRID),
		CLEAR_INTAKE(State.CLEAR_INTAKE),
		BOWLING(State.OUTTAKING_BOWLING);

		public final State outtakeState;

		TargetHeight(final State state) {
			outtakeState = state;
		}
	}

	private Mode m_mode;
	private State m_state;
	private TargetHeight m_height;
	private final LinearFilter m_filter;
	private double m_beamAverage;
	private Optional<Double> m_cubeOffset;

	private Intake() {
		final var positionMotor = MotorControllerFactory.createDefaultTalonFX(POSITION_MOTOR_ID, POSITION_MOTOR_BUS);
		positionMotor.setInvertedOutput(false);
		positionMotor.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);

		m_motorPosition = positionMotor.getInternalController();
		m_motorPosition.config_kP(0, kP);
		m_motorPosition.config_kI(0, 0.0);
		m_motorPosition.config_kD(0, 0.0);
		m_motorPosition.configAllowableClosedloopError(0, State.TICKS_PER_DEGREE * 0.0);
		m_motorPosition.configNeutralDeadband(0.06);

		m_motorPosition.configNominalOutputReverse(-0.03);
		m_motorPosition.configNominalOutputForward(0.03);

		m_motorPosition.configForwardSoftLimitThreshold(State.EXTENDED.ticks);
		m_motorPosition.configReverseSoftLimitThreshold(State.RETRACTED.ticks);
		m_motorPosition.overrideSoftLimitsEnable(true);

//		m_motorPosition.configAllowableClosedloopError(0, State.TICKS_PER_DEGREE * 2.5);

		final var driveMotor = MotorControllerFactory.createDefaultTalonFX(DRIVE_MOTOR_ID, DRIVE_MOTOR_BUS);
		driveMotor.setInvertedOutput(true);
		driveMotor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);

		m_rollerBottom = driveMotor.getInternalController();
		m_rollerBottom.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
				true,
				100,
				100,
				100));

		final var exhaust = MotorControllerFactory.createDefaultTalonFX(16, "rio");
		exhaust.setInvertedOutput(false);
		exhaust.setNeutralBehaviour(NeutralBehaviour.BRAKE);

		m_rollerTop = exhaust.getInternalController();

		m_beamBreak = new DigitalInput(9);
		m_beamAverage = 0.0;
		m_filter = LinearFilter.movingAverage(5);
		m_cubeOffset = Optional.empty();

		m_mode = Mode.DISABLED;
		m_state = State.RETRACTED;
		m_height = TargetHeight.HIGH;
	}

	public boolean isBeamBroken() {
		return m_beamAverage > 0.8; //20
	}

	public void setDisabled() {
		m_mode = Mode.DISABLED;
	}

	public void setState(final State state) {
		m_mode = Mode.POSITION;

		if (state != m_state) {
			m_state = state;
		}
	}

	public void setOuttaking() {
		setState(m_height.outtakeState);
	}

	public void setOuttakeType(final TargetHeight height) {
		m_height = height;
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
		return Math.abs(getError()) < 2000;//State.TICKS_PER_DEGREE * TOLERANCE.getDegrees();
		// return atReference(TOLERANCE);
	}

	public void log() {
		SmartDashboard.putNumber("Intake Amps Drawn", m_rollerBottom.getStatorCurrent());
		SmartDashboard.putNumber("Intake Pos", m_motorPosition.getSelectedSensorPosition());
		SmartDashboard.putNumber("Intake Error", getError());
		SmartDashboard.putNumber("Exhaust Current", m_rollerTop.getStatorCurrent());
		SmartDashboard.putNumber("Beam Broken", m_beamAverage);
	}

	public void setCubeOffset(final double offset) {
		m_cubeOffset = Optional.of(offset);
	}

	public void setCubeOffsetCleared() {
		m_cubeOffset = Optional.empty();
	}

	public Optional<Double> getCubeOffset() {
		return m_cubeOffset;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Intake Offset", m_cubeOffset.orElse(0.0));

		m_beamAverage = m_filter.calculate(!m_beamBreak.get() ? 1.0 : 0.0);

		switch (m_mode) {
			case POSITION:
				m_motorPosition.set(
						ControlMode.Position, m_state.ticks,
						DemandType.ArbitraryFeedForward, 0/*kG * getApproximateAngle().getCos()*/);

				if (!m_state.isOuttake || atReference()) {
					m_rollerBottom.set(ControlMode.PercentOutput, m_state.topDutyCycle);
					m_rollerTop.set(ControlMode.PercentOutput, m_state.bottomDutyCycle);
				} else {
					m_rollerBottom.set(ControlMode.PercentOutput, 0);
					m_rollerTop.set(ControlMode.PercentOutput, 0);
				}

				break;
			case DISABLED:
			default:
				m_motorPosition.set(ControlMode.PercentOutput, 0.0);
				m_rollerBottom.set(ControlMode.PercentOutput, 0.0);
				m_rollerTop.set(ControlMode.PercentOutput, 0.0);
				break;
		}
	}
}
