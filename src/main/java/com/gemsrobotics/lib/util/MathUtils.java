package com.gemsrobotics.lib.util;

public final class MathUtils {
	private MathUtils() {}

	public static double coerce(final double bot, final double n, final double top) {
		if (n > top) {
			return top;
		} else return Math.max(n, bot);
	}
}