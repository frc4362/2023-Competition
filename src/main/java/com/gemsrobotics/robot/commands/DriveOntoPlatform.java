package com.gemsrobotics.robot.commands;

import com.gemsrobotics.lib.util.Translation2dPlus;
import com.gemsrobotics.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;

public class DriveOntoPlatform extends SequentialCommandGroup {
	public DriveOntoPlatform(final Swerve swerve, final Translation2d direction, final double slowScalar) {
		final var pitchMultiplier = Math.signum(direction.getX());
		final var slowD = new Translation2dPlus(direction).normalized().times(slowScalar);

		addCommands(
				new InstantCommand(() -> swerve.setDrivePercent(direction, 0, true, true)),
				swerve.waitForPitchAround(Rotation2d.fromDegrees(Math.copySign(13.0, pitchMultiplier))),
//				new WaitCommand(0.2),
//				swerve.getStopCommand(),
				new InstantCommand(() ->
			      swerve.setDrivePercent(slowD, 0, true, true)),
				swerve.waitForPitchAround(Rotation2d.fromDegrees(Math.copySign(9.5, pitchMultiplier)), 3),
				new InstantCommand(() ->
				  swerve.setDrivePercent(slowD.unaryMinus(), 0, true, true)),
				// new WaitCommand(0.1),
				swerve.getStopCommand()
		);
	}
}
