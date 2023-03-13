package com.gemsrobotics.robot.commands;

import com.gemsrobotics.robot.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WantedStateCommand extends InstantCommand {
	public WantedStateCommand(final Superstructure.WantedState state) {
		super(() -> Superstructure.getInstance().setWantedState(state));
	}
}
