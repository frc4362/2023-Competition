package com.gemsrobotics.robot.commands;

import com.gemsrobotics.lib.LimelightHelpers;
import com.gemsrobotics.lib.util.Rotation2dPlus;
import com.gemsrobotics.lib.util.Units;
import com.gemsrobotics.robot.Constants;
import com.gemsrobotics.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public final class TeleopSwerve extends CommandBase {    
    private static final boolean USE_PLACEMENT_FEEDBACK = true;
    private static final int FILTER_SIZE = 3;
    private static final boolean DO_VISION_ADJUSTMENT = false;
    private static final double VISION_kP = -0.1;

    private final Swerve m_swerve;
    private final DoubleSupplier m_translation;
    private final DoubleSupplier m_strafe;
    private final DoubleSupplier m_rotation;
    private final BooleanSupplier m_isRobotCentric;
    private final BooleanSupplier m_placing;
    private final BooleanSupplier m_pickUping;
    private final BooleanSupplier m_useVision;

    private final SlewRateLimiter m_translationFilter;
    private final SlewRateLimiter m_strafeFilter;
    private final SlewRateLimiter m_rotationFilter;

    public TeleopSwerve(
            final Swerve swerve,
            final DoubleSupplier translationSup,
            final DoubleSupplier strafeSup,
            final DoubleSupplier rotationSup,
            final BooleanSupplier robotCentricSup,
            final BooleanSupplier placementModeSup,
            final BooleanSupplier pickUpLimiter,
            final BooleanSupplier useVisionSup
    ) {
        m_swerve = swerve;
        addRequirements(swerve);

        m_translation = translationSup;
        m_strafe = strafeSup;
        m_rotation = rotationSup;
        m_isRobotCentric = robotCentricSup;
        m_placing = placementModeSup;
        m_pickUping = pickUpLimiter;
        m_useVision = useVisionSup;

        m_translationFilter = new SlewRateLimiter(FILTER_SIZE);
        m_strafeFilter = new SlewRateLimiter(FILTER_SIZE);
        m_rotationFilter = new SlewRateLimiter(FILTER_SIZE);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(m_translation.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(m_strafe.getAsDouble(), Constants.stickDeadband);
        double rotationVal = Math.copySign(Math.pow(MathUtil.applyDeadband(m_rotation.getAsDouble(), Constants.stickDeadband), 2), m_rotation.getAsDouble());

        translationVal = m_translationFilter.calculate(translationVal);
        strafeVal = m_strafeFilter.calculate(strafeVal);
        rotationVal = m_rotationFilter.calculate(rotationVal);

        var translation = new Translation2d(translationVal, strafeVal);
        var nearestPole = new Rotation2dPlus(translation.getAngle()).getNearestPole();

        final double threshold;

        if (m_placing.getAsBoolean()) {
            threshold = 10;
        } else {
            threshold = 5;
        }

        if (Math.abs(translation.getAngle().minus(nearestPole).getDegrees()) < threshold) {
            translation = new Translation2d(translation.getNorm(), nearestPole);
        }

		final double visionError = LimelightHelpers.getTX("");
        final double visionFeedback = VISION_kP * visionError;

        final double placementAngleFeedback = Constants.Generation.thetaController.calculate(
            m_swerve.getYaw().getRadians(),
            Units.degrees2Rads(180)
        );

        final double openLoopRotation = rotationVal * Constants.Swerve.maxAngularVelocity;
        var openLoopTranslation = translation.times(Constants.Swerve.maxSpeed).times((m_placing.getAsBoolean()) ? 0.25 : 1.0);
        openLoopTranslation = translation.times(Constants.Swerve.maxSpeed).times((m_pickUping.getAsBoolean()) ? 0.5 : 1.0); //TODO

        /* Drive */
        m_swerve.setDrive(
            (DO_VISION_ADJUSTMENT && m_useVision.getAsBoolean()) ? new Translation2d(openLoopTranslation.getX(), visionFeedback) : openLoopTranslation,
            (m_placing.getAsBoolean() || m_pickUping.getAsBoolean()) && USE_PLACEMENT_FEEDBACK ? (m_pickUping.getAsBoolean() ? placementAngleFeedback - 180.0 : placementAngleFeedback) : openLoopRotation,
            !m_isRobotCentric.getAsBoolean(),
            true
        );
    }
}
