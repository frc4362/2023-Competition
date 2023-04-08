package com.gemsrobotics.robot.autos;

import java.util.function.Supplier;

import com.gemsrobotics.robot.Constants;
import com.gemsrobotics.robot.commands.CenterOnTagCommand;
import com.gemsrobotics.robot.commands.IntakeUntilCubeCommand;
import com.gemsrobotics.robot.commands.PlaceCommand;
import com.gemsrobotics.robot.commands.ShootCommand;
import com.gemsrobotics.robot.subsystems.*;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ThreeCableAuto extends SequentialCommandGroup {
	public ThreeCableAuto() {
		final var path = PathPlanner.loadPath("States Cable 1", Constants.Generation.constraints);
		final var path2 = PathPlanner.loadPath("States Cable 2", Constants.Generation.constraints);

		final Supplier<Translation2d> allianceBasedStrafe = () -> { 
			return new Translation2d(0.0, 0.25 * getAllianceMultiplier()); 
		};

		addCommands(
				new InstantCommand(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.STARTING)),
				new ParallelCommandGroup(
						Claw.getInstance().requestGrab(),
						new PlaceCommand(SuperstructurePose.AUTON_PLACE).andThen(new WaitCommand(1.0)).andThen(new IntakeUntilCubeCommand(2.0)),
						new WaitCommand(2.00).andThen(Swerve.getInstance().getTrackingCommand(path, true))
				),
				Swerve.getInstance().getStopCommand(),
				new CenterOnTagCommand(this::getAllianceOffset, allianceBasedStrafe),
				Swerve.getInstance().getOdometryResetOnVisionCommand(),
				new ShootCommand(Intake.TargetHeight.HIGH, 0.25),
				new ParallelCommandGroup(
						new WaitCommand(2.00).andThen(new IntakeUntilCubeCommand(3.0)),
						Swerve.getInstance().getTrackingCommand(path2, false)
				),
				new CenterOnTagCommand(this::getAllianceOffset, allianceBasedStrafe),
				new ShootCommand(Intake.TargetHeight.MID, 0.25),
				Swerve.getInstance().getStopCommand()
		);
	}

	private double getAllianceMultiplier() {
		return (DriverStation.getAlliance() == Alliance.Red ? -1.0 : 1.0);
	}

	private double getAllianceOffset() {
		if (DriverStation.getAlliance() == Alliance.Red) {
			return -6.0;
		} else {
			return 6.0;
		}
	}
}
