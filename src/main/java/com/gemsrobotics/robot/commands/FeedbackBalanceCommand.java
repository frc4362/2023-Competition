package com.gemsrobotics.robot.commands;

import com.gemsrobotics.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FeedbackBalanceCommand extends SequentialCommandGroup {
    public FeedbackBalanceCommand() {
        addCommands(
            new InstantCommand(() -> Swerve.getInstance().setDrivePercent(new Translation2d(1.0, 0.0), 0, true, true)),
            Swerve.getInstance().waitForPitchAround(Rotation2d.fromDegrees(13.0)),
            new InstantCommand(() -> Swerve.getInstance().setDrivePercent(new Translation2d(0.25, 0.0), 0, true, true)),
            Swerve.getInstance().waitForPitchAround(Rotation2d.fromDegrees(11.0)),
            Swerve.getInstance().getStopCommand(),
            new InstantCommand(Swerve.getInstance()::setWheelLock)
        );
    }
}
