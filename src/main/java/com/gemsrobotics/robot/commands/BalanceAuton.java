package com.gemsrobotics.robot.commands;

import com.gemsrobotics.robot.Constants;
import com.gemsrobotics.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BalanceAuton extends SequentialCommandGroup {
	public BalanceAuton(final Swerve swerve) {
		addCommands(
				new DriveOntoPlatform(swerve, new Translation2d(-.22, 0.0)),
				new InstantCommand(() -> swerve.setDrivePercent(new Translation2d(-.15, 0.0), 0.0, false, false)),
				new WaitCommand(2.0),
				new DriveOntoPlatform(swerve, new Translation2d(.22, 0.0))
		);
	}
}
