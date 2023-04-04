package com.gemsrobotics.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.led.*;
import com.gemsrobotics.robot.Constants;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.Objects;
import java.util.Optional;

public final class LEDController implements Subsystem {
	private static final boolean IS_INSTALLED = true;

	private static LEDController INSTANCE;

	public static Optional<LEDController> getInstance() {
		if (IS_INSTALLED && Objects.isNull(INSTANCE)) {
			INSTANCE = new LEDController();
		}

		return Optional.ofNullable(INSTANCE);
	}

	private static final int CANDLE_ID = 30;
	private static final String CANDLE_BUS = Constants.CANBusses.MAIN;
	private final Command m_pulseRedCommand;
	private final CANdle m_candle;
//	private final CANifier m_canifier;

	private State m_state;

	private LEDController() {
//		m_canifier = new CANifier(CANDLE_ID);
		m_candle = new CANdle(CANDLE_ID, CANDLE_BUS);

		CANdleConfiguration configAll = new CANdleConfiguration();
		configAll.statusLedOffWhenActive = true;
		configAll.disableWhenLOS = false;
		configAll.stripType = CANdle.LEDStripType.GRB;
		configAll.brightnessScalar = 1.0;
		configAll.vBatOutputMode = CANdle.VBatOutputMode.Off;
		m_candle.configAllSettings(configAll, 100);
		m_candle.clearAnimation(0);

		m_pulseRedCommand = runOnce(() -> setLEDs(Color.kRed))
									.andThen(new WaitCommand(0.2))
									.andThen(() -> setLEDs(Color.kBlack))
									.andThen(new WaitCommand(0.2))
									.andThen(() -> setLEDs(Color.kRed))
									.andThen(new WaitCommand(0.2))
									.andThen(() -> setLEDs(Color.kBlack))
									.andThen(new WaitCommand(0.2))
									.andThen(() -> setLEDs(Color.kRed))
									.andThen(new WaitCommand(0.2));

		m_state = State.OFF;
	}

	public enum State {
		OFF(Color.kBlack),
		WANTS_CONE(Color.kPurple),
		WANTS_SHELF_CUBE(Color.kTeal),
		WANTS_CUBE(Color.kYellow),
		IDLE(Color.kRed);

		public final Color color;

		State(final Color c) {
			color = c;
		}
	}

	public void setState(final State state) {
		m_state = state;
	}

	private void setLEDs(final Color color) {
		m_candle.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
//		m_canifier.setLEDOutput(color.red, CANifier.LEDChannel.LEDChannelA);
//		m_canifier.setLEDOutput(color.green, CANifier.LEDChannel.LEDChannelB);
//		m_canifier.setLEDOutput(color.blue, CANifier.LEDChannel.LEDChannelC);
	}

	private void conformToState() {
		setLEDs(m_state.color);
	}

	@Override
	public Command getDefaultCommand() {
		return run(this::conformToState);
	}

	public void requestPulseRed() {
		m_pulseRedCommand.schedule();
	}

	@Override
	public void periodic() {
		conformToState();
	}
}
