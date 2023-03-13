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
		final var path3 = PathPlanner.loadPath("Test Path 3", Constants.Generation.constraints);

		addCommands(
				new SequentialCommandGroup(
						new InstantCommand(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.STARTING)),
						Claw.getInstance().requestGrab(),
						new ParallelCommandGroup(
								new PlaceCommand(SuperstructurePose.HIGH_PLACE).andThen(new WaitCommand(0.05)).andThen(new IntakeUntilCubeCommand(3.0)),
								new WaitCommand(2.25).andThen(Swerve.getInstance().getTrackingCommand(path, true))
						),
						new ShootCommand(Intake.TargetHeight.HIGH, 0.25),
						new ParallelCommandGroup(
								Swerve.getInstance().getTrackingCommand(path3, false),
								new WaitCommand(1.25).andThen(new IntakeUntilCubeCommand(3.0))
						),
						Swerve.getInstance().getStopCommand(),
						new ShootCommand(Intake.TargetHeight.HIGH, 0.25)
//						new DriveOntoPlatform(Swerve.getInstance(), new Translation2d(-.45, 0.0), .2),
//						new InstantCommand(() -> Swerve.getInstance().setDrivePercent(new Translation2d(0, 0), 0.25, true, false)),
//						new WaitCommand(0.25),
				)
		);
	}
}
