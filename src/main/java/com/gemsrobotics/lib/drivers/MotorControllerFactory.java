package com.gemsrobotics.lib.drivers;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public final class MotorControllerFactory {
    private static final int TIMEOUT_MS = 100;
    public static final String DEFAULT_BUS = "rio";

    private MotorControllerFactory() { }

    public static class TalonConfiguration {
        public NeutralMode NEUTRAL_MODE;
        // factory default
        public double NEUTRAL_DEADBAND;

        public boolean ENABLE_CURRENT_LIMIT;
        public boolean ENABLE_SOFT_LIMIT;
        public boolean ENABLE_LIMIT_SWITCH;
        public int FORWARD_SOFT_LIMIT;
        public int REVERSE_SOFT_LIMIT;

        public boolean INVERTED;
        public boolean SENSOR_PHASE;

        public int CONTROL_FRAME_PERIOD_MS;
        public int MOTION_CONTROL_FRAME_PERIOD_MS;
        public int GENERAL_STATUS_FRAME_RATE_MS;
        public int FEEDBACK_STATUS_FRAME_RATE_MS;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS;

        public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW;

        public double OPEN_LOOP_RAMP_RATE;
        public double CLOSED_LOOP_RAMP_RATE;
    }

    private static final TalonConfiguration DEFAULT_TALON_CONFIG = new TalonConfiguration() {
        {
             NEUTRAL_MODE = NeutralMode.Coast;
             NEUTRAL_DEADBAND = 0.04;

             ENABLE_CURRENT_LIMIT = false;
             ENABLE_SOFT_LIMIT = false;
             ENABLE_LIMIT_SWITCH = false;
             FORWARD_SOFT_LIMIT = 0;
             REVERSE_SOFT_LIMIT = 0;

             INVERTED = false;
             SENSOR_PHASE = false;

             CONTROL_FRAME_PERIOD_MS = 10;
             MOTION_CONTROL_FRAME_PERIOD_MS = 100;
             GENERAL_STATUS_FRAME_RATE_MS = 10;
             FEEDBACK_STATUS_FRAME_RATE_MS = 20;
             QUAD_ENCODER_STATUS_FRAME_RATE_MS = 255;
             ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 255;
             PULSE_WIDTH_STATUS_FRAME_RATE_MS = 255;

             VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_100Ms;
             VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

             OPEN_LOOP_RAMP_RATE = 0.0;
             CLOSED_LOOP_RAMP_RATE = 0.0;
        }
    };

    private static final TalonConfiguration SWERVE_TALON_CONFIG = new TalonConfiguration() {
        {
            NEUTRAL_MODE = NeutralMode.Brake;
            NEUTRAL_DEADBAND = 0.04;

            ENABLE_CURRENT_LIMIT = false;
            ENABLE_SOFT_LIMIT = false;
            ENABLE_LIMIT_SWITCH = false;
            FORWARD_SOFT_LIMIT = 0;
            REVERSE_SOFT_LIMIT = 0;

            INVERTED = false;
            SENSOR_PHASE = false;

            CONTROL_FRAME_PERIOD_MS = 5;
            MOTION_CONTROL_FRAME_PERIOD_MS = 100;
            GENERAL_STATUS_FRAME_RATE_MS = 10;
            FEEDBACK_STATUS_FRAME_RATE_MS = 5;
            QUAD_ENCODER_STATUS_FRAME_RATE_MS = 255;
            ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 255;
            PULSE_WIDTH_STATUS_FRAME_RATE_MS = 255;

            VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_5Ms;
            VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 32;

            OPEN_LOOP_RAMP_RATE = 0.0;
            CLOSED_LOOP_RAMP_RATE = 0.0;
        }
    };

    private static final TalonConfiguration SLAVE_TALON_CONFIG = new TalonConfiguration() {
        {
            NEUTRAL_MODE = NeutralMode.Coast;
            NEUTRAL_DEADBAND = 0.04;

            ENABLE_CURRENT_LIMIT = false;
            ENABLE_SOFT_LIMIT = false;
            ENABLE_LIMIT_SWITCH = false;
            FORWARD_SOFT_LIMIT = 0;
            REVERSE_SOFT_LIMIT = 0;

            INVERTED = false;
            SENSOR_PHASE = false;

            CONTROL_FRAME_PERIOD_MS = 10;
            MOTION_CONTROL_FRAME_PERIOD_MS = 100;
            GENERAL_STATUS_FRAME_RATE_MS = 255;
            FEEDBACK_STATUS_FRAME_RATE_MS = 255;
            QUAD_ENCODER_STATUS_FRAME_RATE_MS = 255;
            ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 255;
            PULSE_WIDTH_STATUS_FRAME_RATE_MS = 255;

            VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_100Ms;
            VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

            OPEN_LOOP_RAMP_RATE = 0.0;
            CLOSED_LOOP_RAMP_RATE = 0.0;
        }
    };

    private static void configureTalon(final TalonFX talon, final TalonConfiguration config) {
        talon.set(ControlMode.PercentOutput, 0.0);

        talon.clearStickyFaults(TIMEOUT_MS);

        talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, TIMEOUT_MS);
        talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, TIMEOUT_MS);
        talon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

        // Turn off re:zeroing by default.
        talon.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, TIMEOUT_MS);
        talon.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, TIMEOUT_MS);

        talon.configNominalOutputForward(0, TIMEOUT_MS);
        talon.configNominalOutputReverse(0, TIMEOUT_MS);
        talon.configNeutralDeadband(config.NEUTRAL_DEADBAND, TIMEOUT_MS);

        talon.configPeakOutputForward(1.0, TIMEOUT_MS);
        talon.configPeakOutputReverse(-1.0, TIMEOUT_MS);

        talon.setNeutralMode(config.NEUTRAL_MODE);

        talon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, TIMEOUT_MS);
        talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, TIMEOUT_MS);

        talon.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, TIMEOUT_MS);
        talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, TIMEOUT_MS);
        talon.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

        talon.setInverted(config.INVERTED);
        talon.setSensorPhase(config.SENSOR_PHASE);

        talon.selectProfileSlot(0, 0);

        talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, TIMEOUT_MS);
        talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, TIMEOUT_MS);

        talon.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, TIMEOUT_MS);
        talon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, TIMEOUT_MS);

        talon.enableVoltageCompensation(false);
        talon.configVoltageCompSaturation(0.0, TIMEOUT_MS);
        talon.configVoltageMeasurementFilter(32, TIMEOUT_MS);

        talon.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, TIMEOUT_MS);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, config.FEEDBACK_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, config.QUAD_ENCODER_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, config.PULSE_WIDTH_STATUS_FRAME_RATE_MS, TIMEOUT_MS);

        talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);
    }

    public static GemTalon<TalonFX> createTalonFX(final int port, final String bus, final TalonConfiguration config, final boolean isSlave) {
        final var talon = new TalonFX(port, bus);
        configureTalon(talon, config);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TIMEOUT_MS);
        return new GemTalon<>(talon, bus, isSlave);
    }

    public static GemTalon<TalonFX> createDefaultTalonFX(final int port, final String bus) {
        return createTalonFX(port, bus, DEFAULT_TALON_CONFIG, false);
    }

    public static GemTalon<TalonFX> createSwerveTalonFX(final int port, final String bus) {
        return createTalonFX(port, bus, SWERVE_TALON_CONFIG, false);
    }

    public static GemTalon<TalonFX> createSlaveTalonFX(final int port, final String bus) {
        return createTalonFX(port, bus, SLAVE_TALON_CONFIG, true);
    }
}
