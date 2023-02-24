package com.gemsrobotics.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmState {
    NULL(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0.0);

    public final Rotation2d wrist;
    public final Rotation2d pivot;
    public final double elevator;

    ArmState(Rotation2d wrist, Rotation2d pivot, double elevator) {
        this.wrist = wrist;
        this.pivot = pivot;
        this.elevator = elevator;
    }
}
