package com.gemsrobotics.robot.subsystems;

public final class SuperstructurePose {
    // TODO
//    public static final SuperstructureState STARTING = new SuperstructureState(Rotation2d.fromDegrees(54), -54.0, 0.025);
//    public static final SuperstructureState
//            // internal pickup states
//            STOWED_FOR_SPINDEXER = null,
//            RECEIVING_FROM_SPINDEXER = null,
//
//            // shelf pickup states
//            STOWED_FOR_SHELF = null,
//            RECEIVING_FROM_WALL = null,
//
//            // floor pickup states
//            RECEIVING_FROM_GROUND = null,
//
//            // scoring states
//            SCORING_HYBRID = null,
//            SCORING_MID = null,
//            SCORING_HIGH = null,
//
//            // general states
//            STOWED = null;

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

    public Pivot.Position getPivot() {
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
}
