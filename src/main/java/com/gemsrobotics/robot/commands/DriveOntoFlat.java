package com.gemsrobotics.robot.commands;

import com.gemsrobotics.lib.util.Translation2dPlus;
import com.gemsrobotics.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class DriveOntoFlat extends SequentialCommandGroup {
	public DriveOntoFlat(final Swerve swerve, final Translation2d direction) {
		final var pitchMultiplier = -Math.signum(direction.getX());
		final var slowD = new Translation2dPlus(direction).normalized().times(0.125);

		addCommands(
				new InstantCommand(() -> swerve.setDrivePercent(direction, 0, false, true)),
				swerve.waitForPitchAround(Rotation2d.fromDegrees(Math.copySign(12.5, pitchMultiplier))),
				new WaitCommand(0.3),
				swerve.getStopCommand(),
				new InstantCommand(() -> swerve.setDrivePercent(slowD, 0, false, true)),
				new WaitUntilCommand(() -> {
					return Math.abs(swerve.getDeltaPitch().getDegrees()) < 1.0 && (Math.abs(swerve.getPitch().getDegrees()) < 1.0);
				})
		);
	}
}
