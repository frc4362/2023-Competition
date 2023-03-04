package com.gemsrobotics.robot.commands;

import com.gemsrobotics.robot.subsystems.Claw;
import com.gemsrobotics.robot.subsystems.Superstructure;
import com.gemsrobotics.robot.subsystems.SuperstructurePose;
import com.gemsrobotics.robot.subsystems.Superstructure.SystemState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public final class SuperstructurePoseCommand extends SequentialCommandGroup {
    public SuperstructurePoseCommand(final SuperstructurePose pose) {
        addCommands(
            new InstantCommand(() -> Superstructure.getInstance().setGoalPose(pose)),
            new WaitUntilCommand(() -> Superstructure.getInstance().getSystemState() == SystemState.ATTAINED_POSE),
            Claw.getInstance().requestDropPiece(),
            new InstantCommand(() -> Superstructure.getInstance().setGoalPoseCleared()),
            new WaitUntilCommand(() -> Superstructure.getInstance().getSystemState() == SystemState.STOWED)
        );
    }
}