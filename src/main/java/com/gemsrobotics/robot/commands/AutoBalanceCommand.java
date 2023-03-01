package com.gemsrobotics.robot.commands;

import com.gemsrobotics.robot.subsystems.Claw;
import com.gemsrobotics.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static com.gemsrobotics.robot.Constants.Generation.getBackwardTrajectory;
import static com.gemsrobotics.robot.Constants.Generation.getForwardTrajectory;
import static java.lang.Math.abs;

public class AutoBalanceCommand extends SequentialCommandGroup {
	public AutoBalanceCommand(final Swerve swerve) {
		final var beachedTrajectory = getBackwardTrajectory(
				new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
				new Pose2d(-1.66, 0, Rotation2d.fromDegrees(0))
		);

		final var flatTrajectory = getBackwardTrajectory(
				new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
				new Pose2d(-2.5, 0, Rotation2d.fromDegrees(0))
		);

//		final var trajectory2 = getBackwardTrajectory(
//				new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
//				new Pose2d(Units.inches2Meters(-60), 0, Rotation2d.fromDegrees(0))
//		);

//		addCommands(
//				Claw.getInstance().setReferenceCommand(Claw.State.OPEN),
//				new WaitCommand(0.25),
//				swerve.getRelativeTrackingCommand(beachedTrajectory),
//				swerve.getStopCommand(),
//				swerve.getWaitUntilBalancedCommand(),
//				Claw.getInstance().setReferenceCommand(Claw.State.GRIPPING),
//				new WaitCommand(0.25),
//				Claw.getInstance().setReferenceCommand(Claw.State.OPEN),
//				new InstantCommand(() -> swerve.setDrivePercent(new Translation2d(-0.28, 0.0), 0, false, true)),
////				new WaitUntilCommand(() -> !swerve.isBalanced()),
//				new WaitUntilCommand(() -> abs(swerve.getPitch().getDegrees()) < 4.0),
//				swerve.getStopCommand(),
//				swerve.getWaitUntilBalancedCommand(),
//				Claw.getInstance().setReferenceCommand(Claw.State.GRIPPING)
//		);
		addCommands(
				new DriveUntilBalancedAgainCommand(swerve, new Translation2d(-.22, 0.0))
		);
	}
}
