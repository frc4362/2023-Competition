package com.gemsrobotics.lib.data;

import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class CachedValue<T> {
	public String getName() {
		return "CachedValue-" + m_type;
	}

	private final double m_timeout;
	private final Class<T> m_type;
	private final Timer m_timer;
	private final Supplier<T> m_source;

	private boolean m_initialized;
	private T m_value, m_oldValue;

	public CachedValue(
			final Class<T> type,
			final double timeout,
			final Supplier<T> source
	) {
		m_type = type;
		m_timeout = timeout;
		m_timer = new Timer();
		m_timer.reset();
		m_source = source;

		m_initialized = false;
		m_value = null;
		m_oldValue = null;
	}

	protected final synchronized void set(final T candidate) {
		m_oldValue = m_value;
		m_value = candidate;
	}

	public final synchronized T get() {
		try {
			if (!m_initialized) {
				set(m_source.get());
				m_initialized = true;
			}

			if (m_timer.hasElapsed(m_timeout)) {
				set(m_source.get());
				m_timer.reset();
			}
		} catch (final Exception ex) {
			System.out.println("Cached value unable to be retrieved: " + ex.getMessage());
			return m_oldValue;
		}

		return m_value;
	}

	public boolean hasExpired() {
	    return m_timer.hasElapsed(m_timeout);
    }

	public synchronized Optional<T> getLastValue() {
		return Optional.ofNullable(m_oldValue);
	}
}
