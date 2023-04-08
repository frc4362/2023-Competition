package com.gemsrobotics.robot.commands;

import com.gemsrobotics.robot.subsystems.Claw;
import com.gemsrobotics.robot.subsystems.Superstructure;
import com.gemsrobotics.robot.subsystems.SuperstructurePose;
import com.gemsrobotics.robot.subsystems.Superstructure.SystemState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public final class PlaceCommand extends SequentialCommandGroup {
    public PlaceCommand(final SuperstructurePose pose) {
        addCommands(
            new InstantCommand(() -> Superstructure.getInstance().setGoalPose(getAppropriatePose(pose))),
            new WaitUntilCommand(() -> Superstructure.getInstance().getSystemState() == SystemState.ATTAINED_POSE),
            Claw.getInstance().requestDropPiece(),
            new InstantCommand(() -> Superstructure.getInstance().setGoalPoseCleared()),
            new WaitUntilCommand(() -> Superstructure.getInstance().getSystemState() == SystemState.STOWED)
        );
    }

    private SuperstructurePose getAppropriatePose(final SuperstructurePose pose) {
        if (pose == SuperstructurePose.AUTON_PLACE && DriverStation.getAlliance() == Alliance.Blue) {
            return SuperstructurePose.AUTON_PLACE_BLUE_STATES;
        } else {
            return pose;
        }
    }
}