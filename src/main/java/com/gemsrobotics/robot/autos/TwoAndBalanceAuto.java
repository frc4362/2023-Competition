package com.gemsrobotics.robot.autos;

import com.gemsrobotics.robot.Constants;
import com.gemsrobotics.robot.commands.*;
import com.gemsrobotics.robot.subsystems.*;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;

public class TwoAndBalanceAuto extends SequentialCommandGroup {
	public TwoAndBalanceAuto() {
		final var path = PathPlanner.loadPath("Test Path", Constants.Generation.constraints);
		final var path2 = PathPlanner.loadPath("Test Path 2", Constants.Generation.constraints);

		addCommands(
				new InstantCommand(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.STARTING)),
				new ParallelCommandGroup(
						Claw.getInstance().requestGrab(),
						new PlaceCommand(SuperstructurePose.AUTON_PLACE).andThen(new WaitCommand(1.)).andThen(new IntakeUntilCubeCommand(2.0)),
						new WaitCommand(2.00).andThen(Swerve.getInstance().getTrackingCommand(path, true))
				),
				new ParallelCommandGroup(
						new ShootCommand(Intake.TargetHeight.HIGH, 0.25),
						new WaitCommand(0.1).andThen(Swerve.getInstance().getTrackingCommand(path2, false)),
						new WaitCommand(1.25).andThen(new IntakeUntilCubeCommand(2.0))
				),
				new FeedbackBalanceCommand(true),
				new InstantCommand(Swerve.getInstance()::setWheelLock)
		);
	}
}
