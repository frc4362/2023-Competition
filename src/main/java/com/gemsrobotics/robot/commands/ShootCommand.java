package com.gemsrobotics.robot.commands;

import com.gemsrobotics.robot.subsystems.Intake;
import com.gemsrobotics.robot.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShootCommand extends SequentialCommandGroup {
	public ShootCommand(final Intake.TargetHeight target, final double duration) {
		addCommands(
				new InstantCommand(() -> Intake.getInstance().setOuttakeType(target)),
				new InstantCommand(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.OUTTAKING)),
				new WaitCommand(duration),
				new InstantCommand(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.STOWED))
		);
	}
}
