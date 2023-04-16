package com.gemsrobotics.robot.commands;

import com.gemsrobotics.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FeedbackBalanceCommand extends SequentialCommandGroup {
    private final boolean m_flipped;

    public FeedbackBalanceCommand(final boolean flipped) {
        m_flipped = flipped;

        addCommands(
            new InstantCommand(() -> Swerve.getInstance().setDrivePercent(new Translation2d(0.6 * getFlipMultiplier(), 0.0), 0, true, true)),
            Swerve.getInstance().waitForPitchAround(Rotation2d.fromDegrees(13 * getFlipMultiplier())),
            new InstantCommand(() -> Swerve.getInstance().setDrivePercent(new Translation2d(0.15 * getFlipMultiplier(), 0.0), 0, true, true)),
            Swerve.getInstance().waitForPitchAround(Rotation2d.fromDegrees(11.3 * getFlipMultiplier())),
            Swerve.getInstance().getStopCommand(),
            new InstantCommand(Swerve.getInstance()::setWheelLock)
        );
    }

    public double getFlipMultiplier() {
        return m_flipped ? -1.0 : 1.0;
    }
}
