package com.gemsrobotics.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

public class SuperstructureState {
    // TODO
    public static final SuperstructureState STARTING = new SuperstructureState(Rotation2d.fromDegrees(54), Rotation2d.fromDegrees(90), 0.0);
    public static final SuperstructureState
            // internal pickup states
            STOWED_FOR_SPINDEXER = null,
            RECEIVING_FROM_SPINDEXER = null,

            // shelf pickup states
            STOWED_FOR_SHELF = null,
            RECEIVING_FROM_WALL = null,

            // floor pickup states
            RECEIVING_FROM_GROUND = null,

            // scoring states
            SCORING_HYBRID = null,
            SCORING_MID = null,
            SCORING_HIGH = null,

            // general states
            STOWED = null;

    public final Rotation2d rotationPivot, rotationWrist;
    public final double elevatatorExtension;

    public SuperstructureState(final Rotation2d pivot, final Rotation2d wrist, final double extension) {
        rotationPivot = pivot;
        rotationWrist = wrist;
        elevatatorExtension = extension;
    }
}