package com.gemsrobotics.robot.subsystems;

import com.gemsrobotics.robot.subsystems.Elevator.Position;

public final class SuperstructurePose {
    public enum Type {
        PICKUP,
        PLACEMENT
    }

    public static final SuperstructurePose SHELF_PICKUP = new SuperstructurePose(
            Type.PICKUP,
            Pivot.Position.SHELF_PICKUP,
            Wrist.Position.SHELF_PICKUP,
            Elevator.Position.SHELF_PICKUP,
            Claw.Goal.OPEN
    );

    public static final SuperstructurePose MID_PLACE = new SuperstructurePose(
            Type.PLACEMENT,
            Pivot.Position.SCORING,
            Wrist.Position.SCORING_MID,
            Elevator.Position.SCORING_MID,
            Claw.Goal.CLOSED
    );

    public static final SuperstructurePose HIGH_PLACE = new SuperstructurePose(
            Type.PLACEMENT,
            Pivot.Position.SCORING,
            Wrist.Position.SCORING_HIGH,
            Elevator.Position.SCORING_HIGH,
            Claw.Goal.CLOSED
    );

    public static final SuperstructurePose AUTON_PLACE = new SuperstructurePose(
        Type.PLACEMENT,
        Pivot.Position.AUTON_SCORING,
        Wrist.Position.SCORING_HIGH,
        Position.AUTON_SCORING_HIGH,
        Claw.Goal.CLOSED
    );

    private final Type m_type;
    private final Pivot.Position m_rotationPivot;
    private final Wrist.Position m_rotationWrist;
    private final Elevator.Position m_elevatorExtension;
    private final Claw.Goal m_clawGoal;

    public SuperstructurePose(
            final Type type,
            final Pivot.Position pivot,
            final Wrist.Position wrist,
            final Elevator.Position extension,
            final Claw.Goal claw
    ) {
        m_type = type;
        m_rotationPivot = pivot;
        m_rotationWrist = wrist;
        m_elevatorExtension = extension;
        m_clawGoal = claw;
    }

    public Type getType() {
        return m_type;
    }

    public Pivot.Position getPivotGoal() {
        return m_rotationPivot;
    }

    public Wrist.Position getWrist() {
        return m_rotationWrist;
    }

    public Elevator.Position getElevator() {
        return m_elevatorExtension;
    }

    public Claw.Goal getClaw() {
        return m_clawGoal;
    }

    public Elevator.Position getElevatorSafety() {
        return Position.FRONT_SAFETY;
        // if (m_type == Type.PICKUP) {
        //     return Elevator.Position.SHELF_PICKUP;
        // } else {
        //     return Elevator.Position.FRONT_SAFETY;
        // }
    }

    public Pivot.Position getPivotSafety() {
        if (m_type == Type.PICKUP) {
            return Pivot.Position.SHELF_PICKUP;
        } else {
            return Pivot.Position.RETURNED;
        }
    }

    public Intake.State getIntake() {
        if (m_type == Type.PICKUP) {
            return Intake.State.RETRACTED;
        } else {
            return Intake.State.MIDDLE;
        }
    }
}
