package com.gemsrobotics.robot.autos;

import com.gemsrobotics.robot.Constants;
import com.gemsrobotics.robot.commands.IntakeUntilCubeCommand;
import com.gemsrobotics.robot.commands.PlaceCommand;
import com.gemsrobotics.robot.commands.ShootCommand;
import com.gemsrobotics.robot.subsystems.*;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.*;

public class TwoAndBalanceAuto extends SequentialCommandGroup {
	public TwoAndBalanceAuto() {
		final var path = PathPlanner.loadPath("Test Path", Constants.Generation.constraints);
		final var path2 = PathPlanner.loadPath("Test Path 2", Constants.Generation.constraints);

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
								Swerve.getInstance().getTrackingCommand(path2, false),
								new WaitCommand(1.25).andThen(new IntakeUntilCubeCommand(3.0))
						),
//						new DriveOntoPlatform(Swerve.getInstance(), new Translation2d(-.45, 0.0), .2),
//						new InstantCommand(() -> Swerve.getInstance().setDrivePercent(new Translation2d(0, 0), 0.25, true, false)),
//						new WaitCommand(0.25),
						Swerve.getInstance().getStopCommand()
				)
		);
	}
}
