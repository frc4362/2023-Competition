package com.gemsrobotics.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;
import java.util.Optional;

public final class Superstructure implements Subsystem {
	private static Superstructure INSTANCE;

	public static Superstructure getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Superstructure();
		}

		return INSTANCE;
	}

	public enum WantedState {
		IDLE,
		STARTING,
		STOWED,
		SCORE
	}

	public enum SystemState {
		IDLE,
		STARTING,
		STOWED,
		WAITING_FOR_INTAKE,
		WAITING_FOR_WRIST,
		WAITING_FOR_EXTENSION,
		READY_TO_SCORE,
		RETURN_TO_CLEAR_ELEVATOR,
		RETURN_TO_CLEAR_PIVOT
	}

	public enum ScoringGoal {
		MID(Elevator.Position.TRUE_BOTTOM),
		HIGH(Elevator.Position.TRUE_BOTTOM);

		public final Elevator.Position extensionTarget;

		ScoringGoal(final Elevator.Position target) {
			extensionTarget = target;
		}
	}

	private final Timer m_stateChangedTimer, m_wantStateChangeTimer;
	private boolean m_stateChanged;
	private WantedState m_stateWanted;
	private SystemState m_state;

	private Optional<ScoringGoal> m_scoringGoal;

	private final Intake m_intake;
	private final Elevator m_elevator;
	private final Pivot m_pivot;
	private final Swerve m_swerve;
	private final Wrist m_wrist;

	private Superstructure() {
		m_intake = Intake.getInstance();
		m_elevator = Elevator.getInstance();
		m_swerve = Swerve.getInstance();
		m_wrist = Wrist.getInstance();
		m_pivot = Pivot.getInstance();

		m_stateChangedTimer = new Timer();
		m_wantStateChangeTimer = new Timer();

		m_state = SystemState.STOWED;
		m_stateWanted = WantedState.STOWED;

		m_scoringGoal = Optional.empty();
	}

	public void setWantedState(final WantedState newState) {
		if (newState != m_stateWanted) {
			m_stateWanted = newState;
			m_wantStateChangeTimer.reset();
		}
	}

	public SystemState getSystemState() {
		return m_state;
	}

	public void setWantScore(final ScoringGoal goal) {
		setWantedState(WantedState.SCORE);
		m_scoringGoal = Optional.of(goal);
	}

	@Override
	public void periodic() {
		SmartDashboard.putString("Wanted State", m_stateWanted.toString());
		SmartDashboard.putString("System State", m_state.toString());

		final SystemState newState;

		switch (m_state) {
			case STOWED:
				newState = handleStowed();
				break;
			case WAITING_FOR_INTAKE:
				newState = handleWaitingForIntakeClear();
				break;
			case WAITING_FOR_WRIST:
				newState = handleWaitingForWristSafety();
				break;
			case WAITING_FOR_EXTENSION:
				newState = handleWaitingForExtension();
				break;
			case READY_TO_SCORE:
				newState = handleReadyToScore();
				break;
			case RETURN_TO_CLEAR_ELEVATOR:
				newState = handleReturnToClearElevator();
				break;
			case RETURN_TO_CLEAR_PIVOT:
				newState = handleReturnToClearPivot();
				break;
			default:
				newState = SystemState.IDLE;
				break;
		}

		if (newState != m_state) {
			m_state = newState;
			m_stateChangedTimer.reset();
			m_stateChanged = true;
		} else {
			m_stateChanged = false;
		}
	}

	private SystemState applyWantedState() {
		switch (m_stateWanted) {
			case STOWED:
				return SystemState.STOWED;
			case SCORE:
				return SystemState.WAITING_FOR_INTAKE;
			default:
				return SystemState.IDLE;
		}
	}

	public SystemState handleIdle() {
		return SystemState.IDLE;
	}

	public SystemState handleStowed() {
		m_intake.setReference(Intake.Position.RETRACTED);
		m_wrist.setReferencePosition(Wrist.Position.STOWED);
		m_elevator.setReference(Elevator.Position.SAFETY_BOTTOM);

		if (m_intake.atReference()) {
			m_pivot.setReference(Pivot.Position.STOWED);
		}

		return applyWantedState();
	}

	public SystemState handleWaitingForIntakeClear() {
		m_intake.setReference(Intake.Position.MIDDLE);

		if (m_intake.atReference()) {
			return SystemState.WAITING_FOR_WRIST;
		} else {
			return applyWantedState();
		}
	}

	public SystemState handleWaitingForWristSafety() {
		m_elevator.setReference(Elevator.Position.FRONT_SAFETY);
		m_wrist.setReferencePosition(Wrist.Position.STOWED);
		m_intake.setReference(Intake.Position.MIDDLE);

		if (m_wrist.isSafeFromElevatorCollision()
			&& m_elevator.atReference()
		) {
			return SystemState.WAITING_FOR_EXTENSION;
		} else if (m_stateWanted == WantedState.SCORE) {
			return SystemState.WAITING_FOR_WRIST;
		} else {
			return applyWantedState();
		}
	}

	public SystemState handleWaitingForExtension() {
		m_elevator.setReference(Elevator.Position.SCORING_MID);
		m_pivot.setReference(Pivot.Position.SCORING);
		m_intake.setReference(Intake.Position.MIDDLE);

		if (m_elevator.atReference() && m_pivot.atReference()) {
			return SystemState.READY_TO_SCORE;
		}

		return SystemState.WAITING_FOR_EXTENSION;
	}

	public SystemState handleReadyToScore() {
		m_intake.setReference(Intake.Position.MIDDLE);
		if (m_stateWanted == WantedState.SCORE) {
			return SystemState.READY_TO_SCORE;
		} else {
			return SystemState.RETURN_TO_CLEAR_ELEVATOR;
		}
	}

	public SystemState handleReturnToClearElevator() {
		m_elevator.setReference(Elevator.Position.FRONT_SAFETY);
		m_pivot.setReference(Pivot.Position.SCORING);
		m_intake.setReference(Intake.Position.MIDDLE);

		if (m_elevator.atReference()) {
			return SystemState.RETURN_TO_CLEAR_PIVOT;
		} else {
			return SystemState.RETURN_TO_CLEAR_ELEVATOR;
		}
	}

	public SystemState handleReturnToClearPivot() {
		m_elevator.setReference(Elevator.Position.SAFETY_BOTTOM);
		m_pivot.setReference(Pivot.Position.RETURNED);
		m_intake.setReference(Intake.Position.MIDDLE);

		if (m_elevator.atReference() && m_pivot.atReference()) {
			return SystemState.STOWED;
		} else {
			return SystemState.RETURN_TO_CLEAR_PIVOT;
		}
	}

	public void stopTimers() {
		m_wantStateChangeTimer.stop();
		m_wantStateChangeTimer.reset();
		m_stateChangedTimer.stop();
		m_stateChangedTimer.reset();
	}
}
