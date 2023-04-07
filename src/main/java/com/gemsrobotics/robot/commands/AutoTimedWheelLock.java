package com.gemsrobotics.robot.commands;

import com.gemsrobotics.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoTimedWheelLock extends SequentialCommandGroup {
    public AutoTimedWheelLock(final Command command, final double duration) {
        addCommands(
            command.withTimeout(duration)
            .andThen(new InstantCommand(Swerve.getInstance()::setWheelLock))
        );
    }
}
