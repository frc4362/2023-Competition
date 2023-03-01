package com.gemsrobotics.robot.commands;

import com.gemsrobotics.lib.util.Translation2dPlus;
import com.gemsrobotics.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class DriveUntilBalancedAgainCommand extends SequentialCommandGroup {
	public DriveUntilBalancedAgainCommand(final Swerve swerve, final Translation2d direction) {
		addCommands(
				new InstantCommand(() -> swerve.setDrivePercent(direction, 0, false, true)),
				swerve.waitForPitchAround(Rotation2d.fromDegrees(-12.0)),
				new InstantCommand(() ->
				   swerve.setDrivePercent(new Translation2dPlus(direction).normalized().times(0.15), 0, false, true)),
				swerve.waitForPitchAround(Rotation2d.fromDegrees(0.0), 3),
				swerve.getStopCommand()
		);
	}
}
