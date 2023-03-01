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
		ATTAIN_POSE
	}

	public enum SystemState {
		IDLE,
		STARTING,
		STOWED,
		WAITING_FOR_INTAKE,
		WAITING_FOR_WRIST,
		EXTENDING_FOR_FRONT_POSE,
		READY_FOR_FRONT_POSE,
		ATTAINED_POSE,
		RETURN_TO_CLEAR_ELEVATOR,
		RETURN_TO_CLEAR_PIVOT
	}

	private final Timer m_stateChangedTimer, m_wantStateChangeTimer;
	private boolean m_stateChanged;
	private WantedState m_stateWanted;
	private SystemState m_state;
	private Optional<SuperstructurePose> m_poseGoal;

	private final Intake m_intake;
	private final Elevator m_elevator;
	private final Pivot m_pivot;
	private final Swerve m_swerve;
	private final Wrist m_wrist;
	private final Claw m_claw;

	private Superstructure() {
		m_intake = Intake.getInstance();
		m_elevator = Elevator.getInstance();
		m_swerve = Swerve.getInstance();
		m_wrist = Wrist.getInstance();
		m_pivot = Pivot.getInstance();
		m_claw = Claw.getInstance();

		m_stateChangedTimer = new Timer();
		m_wantStateChangeTimer = new Timer();

		m_state = SystemState.STOWED;
		m_stateWanted = WantedState.STOWED;

		m_poseGoal = Optional.empty();
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

	public void setGoalPose(final SuperstructurePose goal) {
		setWantedState(WantedState.ATTAIN_POSE);
		m_poseGoal = Optional.of(goal);
	}

	public void setGoalPoseCleared() {
		setWantedState(WantedState.STOWED);
		m_poseGoal = Optional.empty();
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
				newState = handleWaitingForIntakeSafety();
				break;
			case WAITING_FOR_WRIST:
				newState = handleWaitingForWristSafety();
				break;
			case READY_FOR_FRONT_POSE:
				newState = handleReadyForFrontPose();
				break;
			case EXTENDING_FOR_FRONT_POSE:
				newState = handleExtendingForFrontPose();
				break;
			case ATTAINED_POSE:
				newState = handleAttainedPose();
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
			case ATTAIN_POSE:
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
		m_claw.setIdealStowedGoal();

		if (m_intake.atReference()) {
			m_pivot.setReference(Pivot.Position.STOWED);
		}

		return applyWantedState();
	}

	public SystemState handleWaitingForIntakeSafety() {
		m_pivot.setReference(Pivot.Position.RETURNED);
		m_intake.setReference(Intake.Position.MIDDLE);
		m_claw.setGoal(m_poseGoal.map(SuperstructurePose::getClaw).orElse(Claw.Goal.CLOSED));

		if (m_intake.atReference()) {
			return SystemState.WAITING_FOR_WRIST;
		} else {
			return applyWantedState();
		}
	}

	public SystemState handleWaitingForWristSafety() {
		m_pivot.setReference(Pivot.Position.STOWED);
		m_elevator.setReference(Elevator.Position.FRONT_SAFETY);
		m_wrist.setReferencePosition(Wrist.Position.CLEAR);
		m_intake.setReference(Intake.Position.MIDDLE);

		if (m_wrist.isSafeFromElevatorCollision()
			&& m_elevator.atReference()
		) {
			return SystemState.EXTENDING_FOR_FRONT_POSE;//SystemState.WAITING_FOR_EXTENSION;
		} else if (m_stateWanted == WantedState.ATTAIN_POSE) {
			return SystemState.WAITING_FOR_WRIST;
		} else {
			return applyWantedState();
		}
	}

	private SystemState handleExtendingForFrontPose() {
		m_elevator.setReference(m_poseGoal.map(SuperstructurePose::getElevator).orElse(Elevator.Position.FRONT_SAFETY));

		if (m_elevator.atReference()) {
			return SystemState.READY_FOR_FRONT_POSE;
		} else {
			return SystemState.EXTENDING_FOR_FRONT_POSE;
		}
	}

	public SystemState handleReadyForFrontPose() {
		m_elevator.setReference(m_poseGoal.map(SuperstructurePose::getElevator).orElse(Elevator.Position.FRONT_SAFETY));
		m_pivot.setReference(m_poseGoal.map(SuperstructurePose::getPivot).orElse(Pivot.Position.STOWED));
		m_wrist.setReferencePosition(m_poseGoal.map(SuperstructurePose::getWrist).orElse(Wrist.Position.CLEAR));
		m_claw.setGoal(m_poseGoal.map(SuperstructurePose::getClaw).orElse(Claw.Goal.CLOSED));

		if (m_elevator.atReference() && m_pivot.atReference() && m_wrist.atReference()) {
			return SystemState.ATTAINED_POSE;
		}

		return SystemState.READY_FOR_FRONT_POSE;
	}

	public SystemState handleAttainedPose() {
		if (m_stateWanted == WantedState.ATTAIN_POSE) {
			if (m_poseGoal.map(SuperstructurePose::getType)
						.map(type -> type == SuperstructurePose.Type.PICKUP)
						.orElse(false)
				&& m_claw.getObservedPiece().isPresent() && m_claw.getPieceConfidence()
			) {
				setWantedState(WantedState.STOWED);
				return SystemState.RETURN_TO_CLEAR_ELEVATOR;
			}

			return SystemState.ATTAINED_POSE;
		} else {
			return SystemState.RETURN_TO_CLEAR_ELEVATOR;
		}
	}

	public SystemState handleReturnToClearElevator() {
		m_elevator.setReference(Elevator.Position.FRONT_SAFETY);
		m_pivot.setReference(Pivot.Position.STOWED);
		m_wrist.setReferencePosition(Wrist.Position.CLEAR);

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
		m_wrist.setReferencePosition(Wrist.Position.STOWED);

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