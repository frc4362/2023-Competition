package com.gemsrobotics.robot.autos;

import com.gemsrobotics.robot.Constants;
import com.gemsrobotics.robot.commands.IntakeUntilCubeCommand;
import com.gemsrobotics.robot.commands.PlaceCommand;
import com.gemsrobotics.robot.commands.ShootCommand;
import com.gemsrobotics.robot.subsystems.*;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ThreeAuto extends SequentialCommandGroup {
	public ThreeAuto() {
		final var path = PathPlanner.loadPath("Test Path", Constants.Generation.constraints);
		final var path3 = PathPlanner.loadPath("Test Path 3 Alternative", Constants.Generation.constraints);

		addCommands(
			new InstantCommand(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.STARTING)),
			new ParallelCommandGroup(
					Claw.getInstance().requestGrab(),
					new PlaceCommand(SuperstructurePose.AUTON_PLACE).andThen(new WaitCommand(0.5)).andThen(new IntakeUntilCubeCommand(2.0)),
					new WaitCommand(2.00).andThen(Swerve.getInstance().getTrackingCommand(path, true))
			),
			new ParallelCommandGroup(
					new ShootCommand(Intake.TargetHeight.HIGH, 0.25),
//					Swerve.getInstance().getOdometryResetOnVisionCommand().andThen(
					new WaitCommand(0.05).andThen(Swerve.getInstance().getTrackingCommand(path3, false)),
					new WaitCommand(1.2).andThen(new IntakeUntilCubeCommand(2.0))
			),
			new ShootCommand(Intake.TargetHeight.HIGH_AUTO, 0.05)
//						new DriveOntoPlatform(Swerve.getInstance(), new Translation2d(-.45, 0.0), .2),
//						new InstantCommand(() -> Swerve.getInstance().setDrivePercent(new Translation2d(0, 0), 0.25, true, false)),
//						new WaitCommand(0.25),
		);
	}
}
