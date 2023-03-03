package com.gemsrobotics.robot.subsystems;

import java.util.Objects;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.MotorController;
import com.gemsrobotics.lib.drivers.MotorControllerFactory;
import com.gemsrobotics.robot.Constants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public final class Claw implements Subsystem {
	private static final boolean DO_LOGGING = false;
	public static final double CONE_THRESHOLD_ROTATIONS = -.31;

	public static final double PIECE_DETECTION_TIME = 0.5;
	public static final int PIECE_GRIPPED_THRESHOLD_AMPS = 35;
	public static final double NOTHING_THRESHOLD_ROTATIONS = -.35;

	private static Claw INSTANCE = null;

	private static final int CLAW_DRIVE_MOTOR_ID = 19;
	private static final String CLAW_DRIVE_MOTOR_BUS = Constants.CANBusses.MAIN;
	private static final int CLAW_GRIP_MOTOR_ID = 14;
	private static final String CLAW_GRIP_MOTOR_BUS = Constants.CANBusses.MAIN;

	private static final double TOLERANCE = 0.05;

	private static final double GEARING_MULTIPLIER = 1.0 / 10.5;

	private static final double GRIP_CURRENT_LIMIT = 10.0;
	private static final double GRIP_DRIVE_VOLTAGE = 2.0;
	private static final double GRIP_OPEN_POSITION = -0.24;//TODO

	private static final double CLAW_DRIVE_VOLTAGE = 2.0;

	/*We liked running -1 Volts at 40A */
	private final MotorController<TalonFX> m_motorDrive;
	private final MotorController<TalonFX> m_motorGrip;
	private final Timer m_pieceTimer;

	private Goal m_goal;
	private IntakeState m_intakeState;
	private boolean m_forceGrip;

	public static Claw getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Claw();
		}

		return INSTANCE;
	}

	private Claw() {
		m_motorDrive = MotorControllerFactory.createDefaultTalonFX(CLAW_DRIVE_MOTOR_ID, CLAW_DRIVE_MOTOR_BUS);
		m_motorDrive.setInvertedOutput(true);
		m_motorDrive.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);

		m_motorGrip = MotorControllerFactory.createDefaultTalonFX(CLAW_GRIP_MOTOR_ID, CLAW_GRIP_MOTOR_BUS);
		m_motorGrip.setInvertedOutput(true);
		m_motorGrip.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motorGrip.setGearingParameters(GEARING_MULTIPLIER, 1.0);
		m_motorGrip.setOpenLoopVoltageRampRate(0.6);

		m_motorGrip.getInternalController().configStatorCurrentLimit(
				new StatorCurrentLimitConfiguration(true, 40, 40, 5));

		m_pieceTimer = new Timer();
		m_pieceTimer.reset();
		m_pieceTimer.stop();

		m_goal = Goal.OPEN;
		m_intakeState = IntakeState.NEUTRAL;
		m_forceGrip = false;
	}

	public enum Goal {
		NEUTRAL,
		CLOSED,
		GRIPPING,
		OPEN
	}

	public enum IntakeState {
		NEUTRAL(0.0),
		INTAKING(0.2),
		OUTTAKING(-0.5);

		public final double dutyCycle;

		IntakeState(final double d) {
			dutyCycle = d;
		}
	}

	public enum ObservedPiece {
		CUBE,
		CONE
	}

	public void setGoal(final Goal state) {
		m_goal = state;
	}

	public Command requestDropPiece() {
		return runOnce(() -> {
			setGoal(Goal.OPEN);
			setForceGrip(false);
		});
	}

	public void setForceGrip(final boolean grip) {
		m_forceGrip = grip;
	}

	public double getPosition() {
		return m_motorDrive.getPositionRotations();
	}

	public Optional<ObservedPiece> getObservedPiece() {
		if (Math.abs(m_motorGrip.getDrawnCurrentAmps()) < PIECE_GRIPPED_THRESHOLD_AMPS) {
			return Optional.empty();
		}

		if (m_motorGrip.getPositionRotations() < NOTHING_THRESHOLD_ROTATIONS) {
			return Optional.empty();
		} else if (m_motorGrip.getPositionRotations() < CONE_THRESHOLD_ROTATIONS) {
			return Optional.of(ObservedPiece.CONE);
		} else {
			return Optional.of(ObservedPiece.CUBE);
		}
	}

	public boolean getPieceConfidence() {
		return m_pieceTimer.get() > PIECE_DETECTION_TIME;
	}

	public void setIdealStowedGoal() {
		if (m_forceGrip || (getObservedPiece().isPresent() && getPieceConfidence())) {
			setGoal(Goal.GRIPPING);
		} else {
			setGoal(Goal.CLOSED);
		}
	}

	public void setIntakeState(final IntakeState state) {
		m_intakeState = state;
	}

	public Command requestGrab() {
		return runOnce(
				() -> {
					setGoal(Goal.GRIPPING);
					setIntakeState(IntakeState.INTAKING);
				})
					   .andThen(new WaitUntilCommand(() -> getObservedPiece().isPresent() && getPieceConfidence())
										.raceWith(new WaitCommand(2.5).andThen(() -> {
											setGoal(Goal.OPEN);
											LEDController.getInstance().ifPresent(LEDController::requestPulseRed);
										})))
					   .finallyDo(interrupted -> setIntakeState(IntakeState.NEUTRAL));
	}

	public void log() {
		SmartDashboard.putNumber("Claw Grip Rotations", m_motorGrip.getPositionRotations());
		SmartDashboard.putNumber("Claw Grip Drawn Amps", m_motorGrip.getDrawnCurrentAmps());
//		SmartDashboard.putNumber("Claw Grip Control Effort", m_motorGrip.getVoltageOutput());
		SmartDashboard.putString("Claw Observed Piece", getObservedPiece().map(ObservedPiece::toString).orElse("NONE"));
//		SmartDashboard.putBoolean("Claw Force Closed", m_forceGrip);
//		SmartDashboard.putString("Claw State", m_goal.toString());
	}

	@Override
	public void periodic() {
		final var observedPiece = getObservedPiece();

		m_motorDrive.setDutyCycle(m_intakeState.dutyCycle);

		if (observedPiece.isPresent()) {
			m_pieceTimer.start();
		} else {
			m_pieceTimer.reset();
			m_pieceTimer.stop();
		}

		switch (m_goal) {
			case GRIPPING:
				m_motorGrip.setVoltage(-1.0);

				setForceGrip(getPieceConfidence());
				break;

			case CLOSED:
				if (m_forceGrip || m_motorGrip.getPositionRotations() > CONE_THRESHOLD_ROTATIONS) {
					m_motorGrip.setVoltage(-1.0);
				} else {
					m_motorGrip.setVoltage(0.0);
				}
				break;

			case OPEN:
				if (m_forceGrip) {
					m_motorGrip.setVoltage(-1.0);
				} else if (m_motorGrip.getPositionRotations() < GRIP_OPEN_POSITION) {
					m_motorGrip.setVoltage(2.0);
				} else {
					m_motorGrip.setVoltage(0.0);
				}
				break;

			case NEUTRAL:
				m_motorGrip.setVoltage(0.0);
				break;
		}
	}
}
