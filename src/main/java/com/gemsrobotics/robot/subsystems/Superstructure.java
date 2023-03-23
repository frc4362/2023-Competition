package com.gemsrobotics.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;
import java.util.Optional;

import com.gemsrobotics.robot.subsystems.Intake.State;
import com.gemsrobotics.robot.subsystems.Pivot.Position;
import com.gemsrobotics.robot.subsystems.SuperstructurePose.Type;

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
		ATTAIN_POSE,
		INTAKING,
		OUTTAKING,
		HYBRID,
		CENTERING
	}

	public enum SystemState {
		IDLE,
		STARTING,

		// motion states
		STOWED,
		WAITING_FOR_INTAKE,
		WAITING_FOR_WRIST,
		EXTENDING_FOR_FRONT_POSE,
		READY_FOR_FRONT_POSE,
		ATTAINED_POSE,
		RETURN_TO_CLEAR_ELEVATOR,
		RETURN_TO_CLEAR_PIVOT,
		HYBRID_PLACEMENT,

		// intaking states
		INTAKING,
		OUTTAKING,
		CENTERING
	}

	private final Timer m_stateChangedTimer, m_wantStateChangeTimer;
	private boolean m_stateChanged;
	private WantedState m_stateWanted;
	private SystemState m_state;
	private Optional<SuperstructurePose> m_poseGoal;
	private boolean m_wantsHat;

	private final Intake m_intake;
	private final Elevator m_elevator;
	private final Pivot m_pivot;
	private final Swerve m_swerve;
	private final Wrist m_wrist;
	private final Claw m_claw;

	private boolean m_shelfPickUpLimiter;

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

		m_wantsHat = false;
		m_shelfPickUpLimiter = false;
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

	public boolean hasScoringGoal() {
		return m_poseGoal.isPresent();
	}

	public void setGoalPose(final SuperstructurePose goal) {
		setWantedState(WantedState.ATTAIN_POSE);
		m_poseGoal = Optional.of(goal);
	}

	public void setGoalPoseCleared() {
		setWantedState(WantedState.STOWED);
		m_poseGoal = Optional.empty();
	}

	public void setDoHat(final boolean h) {
		m_wantsHat = h;
	}

	public boolean isShelfPickupLimiter() {
		return m_shelfPickUpLimiter;
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

			// motion states
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

			// other states
			case INTAKING:
				newState = handleIntaking();
				break;
			case OUTTAKING:
				newState = handleOuttaking();
				break;
			case HYBRID_PLACEMENT:
				newState = handlePlacingHybrid();
				break;
			case STARTING:
				newState = handleStarting();
				break;
			case CENTERING:
				newState = handleCentering();
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

		//On pickup of peice turn off limiter
		if(m_shelfPickUpLimiter == true && null != m_claw.getObservedPiece().orElse(null)) {
			m_shelfPickUpLimiter = false;
		}

	}

	private SystemState applyWantedState() {
		switch (m_stateWanted) {
			case STOWED:
				return SystemState.STOWED;
			case ATTAIN_POSE:
				return SystemState.WAITING_FOR_INTAKE;
			case INTAKING:
				return SystemState.INTAKING;
			case OUTTAKING:
				return SystemState.OUTTAKING;
			case HYBRID:
				return SystemState.HYBRID_PLACEMENT;
			case STARTING:
				return SystemState.STARTING;
			case CENTERING:
				return SystemState.CENTERING;
			default:
				return SystemState.IDLE;
		}
	}

	public SystemState handleIdle() {
		return SystemState.IDLE;
	}

	private SystemState handleStowed() {
		if (m_stateChanged) {
			setGoalPoseCleared();
		}

		m_intake.setState(Intake.State.RETRACTED);
		m_wrist.setReferencePosition(Wrist.Position.STOWED);
		m_elevator.setReference(Elevator.Position.SAFETY_BOTTOM);
		// m_claw.setIdealStowedGoal();

		m_shelfPickUpLimiter = false; // TODO probaly make this more intentional

		if (m_intake.atReference()) {
			m_pivot.setReference(Pivot.Position.STOWED);
		}

		return applyWantedState();
	}

	private SystemState handleStarting() {
		m_wrist.setReferencePosition(Wrist.Position.PRELOAD);

		return applyWantedState();
	}

	private SystemState handleCentering() {
		m_intake.setState(State.CENTER);

		return applyWantedState();
	}

	private SystemState handlePlacingHybrid() {
		m_pivot.setReference(Pivot.Position.STOWED);


		if (m_claw.getObservedPiece().map(piece -> piece == Claw.ObservedPiece.CUBE).orElse(false)) {
			m_wrist.setReferencePosition(Wrist.Position.HYBRID_CUBE);
		} else {
			m_wrist.setReferencePosition(Wrist.Position.HYBRID_CONE);
		}

		return applyWantedState();
	}

	private SystemState handleWaitingForIntakeSafety() {
		m_pivot.setReference(m_poseGoal.map(SuperstructurePose::getPivotSafety).orElse(Pivot.Position.STOWED));
		m_intake.setState(m_poseGoal.map(SuperstructurePose::getIntake).orElse(Intake.State.RETRACTED));
		m_claw.setGoal(m_poseGoal.map(SuperstructurePose::getClaw).orElse(Claw.Goal.CLOSED));

		if (m_intake.atReference() || m_poseGoal.map(SuperstructurePose::getType).map(type -> type == Type.PICKUP).orElse(false)) {
			return SystemState.WAITING_FOR_WRIST;
		} else {
			return applyWantedState();
		}
	}

	private SystemState handleWaitingForWristSafety() {
		m_pivot.setReference(m_poseGoal.map(SuperstructurePose::getPivotGoal).orElse(Pivot.Position.STOWED));
		m_elevator.setReference(m_poseGoal.map(SuperstructurePose::getElevatorSafety).orElse(Elevator.Position.FRONT_SAFETY));
		m_wrist.setReferencePosition(Wrist.Position.CLEAR);
		m_intake.setState(m_poseGoal.map(SuperstructurePose::getIntake).orElse(Intake.State.RETRACTED));
		m_claw.setGoal(m_poseGoal.map(SuperstructurePose::getClaw).orElse(Claw.Goal.CLOSED));

		m_shelfPickUpLimiter = m_poseGoal.map(SuperstructurePose::getType).map(type -> type == Type.PICKUP).orElse(false);

		if (m_wrist.isSafeFromElevatorCollision()
			&& m_elevator.atReference()
		) {
			return SystemState.READY_FOR_FRONT_POSE;//SystemState.WAITING_FOR_EXTENSION;
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

	private SystemState handleReadyForFrontPose() {
		m_elevator.setReference(m_poseGoal.map(SuperstructurePose::getElevator).orElse(Elevator.Position.SAFETY_BOTTOM));
		m_pivot.setReference(m_poseGoal.map(SuperstructurePose::getPivotGoal).orElse(Pivot.Position.STOWED));
		m_wrist.setReferencePosition(m_poseGoal.map(SuperstructurePose::getWrist).orElse(Wrist.Position.CLEAR));
		m_claw.setGoal(m_poseGoal.map(SuperstructurePose::getClaw).orElse(Claw.Goal.CLOSED));

		// if (m_elevator.getHeightMeters() > 0.41 && m_elevator.getReferenceMeters() > Elevator.Position.FRONT_SAFETY.extensionMeters) {
		// 	m_intake.setState(State.RETRACTED);
		// }

		if (m_elevator.atReference() && m_pivot.atReference() && m_wrist.atReference(5.0)) {
			return SystemState.ATTAINED_POSE;
		}

		return SystemState.READY_FOR_FRONT_POSE;
	}

	private SystemState handleAttainedPose() {
		final var observedPiece = m_claw.getObservedPiece().orElse(null);

		if (m_poseGoal.map(SuperstructurePose::getType).map(pose -> pose == Type.PLACEMENT).orElse(false)) {
			final Wrist.Position wristPosition;

			if (m_wantsHat) {
				if (observedPiece == Claw.ObservedPiece.CUBE) {
					wristPosition = Wrist.Position.HAT;
				} else if (observedPiece == Claw.ObservedPiece.CONE) {
					wristPosition = Wrist.Position.HALF_HAT;
				} else {
					wristPosition = m_poseGoal.get().getWrist();
				}
			} else {
				wristPosition = m_poseGoal.get().getWrist();
			}

			m_wrist.setReferencePosition(wristPosition);
			m_pivot.setReference(m_wantsHat && observedPiece == Claw.ObservedPiece.CONE
								 ? Pivot.Position.HATTING : m_poseGoal.get().getPivotGoal());
		} else {
			m_wrist.setReferencePosition(m_poseGoal.map(SuperstructurePose::getWrist).orElse(Wrist.Position.CLEAR));
			m_pivot.setReference(m_poseGoal.map(SuperstructurePose::getPivotGoal).orElse(Position.SHELF_PICKUP));
		}

		if (m_stateWanted == WantedState.ATTAIN_POSE) {
			final var poseType = m_poseGoal.map(SuperstructurePose::getType);
			if (poseType.map(type -> type == SuperstructurePose.Type.PICKUP).orElse(false)
				&& m_claw.getObservedPiece().isPresent() && m_claw.getPieceConfidence()
			) {
				// LEDController.getInstance().ifPresent(controller -> controller.setState(LEDController.State.OFF));
				setWantedState(WantedState.STOWED);
				return SystemState.RETURN_TO_CLEAR_ELEVATOR;
			} else if (poseType.map(type -> type == SuperstructurePose.Type.PLACEMENT).orElse(false)
						&& m_claw.getObservedPiece().isEmpty()
			) {
				setWantedState(WantedState.STOWED);
				return SystemState.RETURN_TO_CLEAR_ELEVATOR;
			}

			return SystemState.ATTAINED_POSE;
		} else {
			return SystemState.RETURN_TO_CLEAR_ELEVATOR;
		}
	}

	private SystemState handleReturnToClearElevator() {
		m_elevator.setReference(m_poseGoal.map(SuperstructurePose::getElevatorSafety).orElse(Elevator.Position.SAFETY_BOTTOM));
		m_intake.setState(m_poseGoal.map(SuperstructurePose::getIntake).orElse(Intake.State.RETRACTED));
//		m_pivot.setReference(Pivot.Position.STOWED);
		m_wrist.setReferencePosition(Wrist.Position.CLEAR);

		if (m_elevator.atReference()) {
			return SystemState.RETURN_TO_CLEAR_PIVOT;
		} else {
			return SystemState.RETURN_TO_CLEAR_ELEVATOR;
		}
	}

	private SystemState handleReturnToClearPivot() {
		m_elevator.setReference(Elevator.Position.SAFETY_BOTTOM);
		m_pivot.setReference(m_poseGoal.map(SuperstructurePose::getPivotSafety).orElse(Pivot.Position.RETURNED));
		m_intake.setState(m_poseGoal.map(SuperstructurePose::getIntake).orElse(Intake.State.RETRACTED));
		m_wrist.setReferencePosition(Wrist.Position.STOWED);

		if (m_elevator.atReference() && m_pivot.atReference()) {
			return SystemState.STOWED;
		} else {
			return SystemState.RETURN_TO_CLEAR_PIVOT;
		}
	}

	private SystemState handleIntaking() {
		m_pivot.setReference(Position.RETURNED);
		m_intake.setState(State.INTAKING);

		return applyWantedState();
	}

	private SystemState handleOuttaking() {
		m_pivot.setReference(Position.STOWED);
		m_intake.setOuttaking();

		return applyWantedState();
	}

	public void stopTimers() {
		m_wantStateChangeTimer.stop();
		m_wantStateChangeTimer.reset();
		m_stateChangedTimer.stop();
		m_stateChangedTimer.reset();
	}
}
