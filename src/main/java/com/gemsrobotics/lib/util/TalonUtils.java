package com.gemsrobotics.lib.util;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TalonUtils {
	private TalonUtils() {
	}

	public static boolean isEncoderPresent(final TalonSRX device) {
		return device.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
	}
}
