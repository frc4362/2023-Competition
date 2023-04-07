package com.gemsrobotics.robot.commands;

import com.gemsrobotics.lib.LimelightHelpers;
import com.gemsrobotics.robot.subsystems.Intake;
import com.gemsrobotics.robot.subsystems.LEDController;
import com.gemsrobotics.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;

public class IntakeUntilCubeCommand extends SequentialCommandGroup {
	public IntakeUntilCubeCommand(final double timeout) {
		addCommands(
				new WantedStateCommand(Superstructure.WantedState.INTAKING),
				new RunCommand(() -> Intake.getInstance().setCubeOffset(LimelightHelpers.getTX("")))
						.until(Intake.getInstance()::isBeamBroken)
						.withTimeout(timeout)
						.finallyDo(interrupted -> {
								Superstructure.getInstance().setWantedState(Superstructure.WantedState.STOWED);
								
								if (!interrupted) {
									LEDController.getInstance()
										.map(controller -> controller.requestPulseCommand(Color.kPurple))
										.ifPresent(Command::schedule);
								}
						})
		);
	}
}
