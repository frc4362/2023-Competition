package com.gemsrobotics.robot.commands;

import com.gemsrobotics.robot.subsystems.Intake;
import com.gemsrobotics.robot.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class IntakeUntilCubeCommand extends SequentialCommandGroup {
	public final boolean ATTEMPT_CENTERING = false;

	public IntakeUntilCubeCommand(final double timeout) {
		addCommands(
				new WantedStateCommand(Superstructure.WantedState.INTAKING),
				new WaitUntilCommand(Intake.getInstance()::isBeamBroken)
						.withTimeout(timeout)
						.finallyDo(interrupted -> {
							if (!ATTEMPT_CENTERING || interrupted) {
								Superstructure.getInstance().setWantedState(Superstructure.WantedState.STOWED);
							} else {
								Superstructure.getInstance().setWantedState(Superstructure.WantedState.CENTERING);
							}
						}),
				new WaitCommand(ATTEMPT_CENTERING ? 1.0 : 0.0)
		);
	}
}
