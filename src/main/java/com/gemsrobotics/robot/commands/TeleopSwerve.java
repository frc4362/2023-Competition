package com.gemsrobotics.robot.commands;

import com.gemsrobotics.lib.LimelightHelpers;
import com.gemsrobotics.lib.util.Rotation2dPlus;
import com.gemsrobotics.lib.util.Units;
import com.gemsrobotics.robot.Constants;
import com.gemsrobotics.robot.subsystems.Superstructure;
import com.gemsrobotics.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public final class TeleopSwerve extends CommandBase {    
    private static final boolean USE_PLACEMENT_FEEDBACK = true;
    private static final int FILTER_SIZE = 3;
    private static final boolean DO_VISION_ADJUSTMENT = true;
    private static final double VISION_kP = 1.0 / 40.0;

    private final Swerve m_swerve;
    private final DoubleSupplier m_translation;
    private final DoubleSupplier m_strafe;
    private final DoubleSupplier m_rotation;
    private final BooleanSupplier m_isRobotCentric;
    private final BooleanSupplier m_placing;
    private final BooleanSupplier m_pickUping;
    private final BooleanSupplier m_useVision;
    private final BooleanSupplier m_doEvasionSup;
    private final BooleanSupplier m_doCubeYawLock;

    private final SlewRateLimiter m_translationFilter;
    private final SlewRateLimiter m_strafeFilter;
    private final SlewRateLimiter m_rotationFilter;

    private double m_startPickupTime;

    public TeleopSwerve(
            final Swerve swerve,
            final DoubleSupplier translationSup,
            final DoubleSupplier strafeSup,
            final DoubleSupplier rotationSup,
            final BooleanSupplier robotCentricSup,
            final BooleanSupplier placementModeSup,
            final BooleanSupplier pickUpLimiter,
            final BooleanSupplier useVisionSup,
            final BooleanSupplier doEvasionSup,
            final BooleanSupplier doCubeYawLock
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
        m_doEvasionSup = doEvasionSup;
        m_doCubeYawLock = doCubeYawLock;

        m_translationFilter = new SlewRateLimiter(FILTER_SIZE);
        m_strafeFilter = new SlewRateLimiter(FILTER_SIZE);
        m_rotationFilter = new SlewRateLimiter(FILTER_SIZE);

        m_startPickupTime = 0.0;
    }

    @Override
    public void execute() {
        // m_swerve.setWheelLock();
        // return;

        if (m_pickUping.getAsBoolean()) {
            m_startPickupTime = Timer.getFPGATimestamp();
        }

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

        final double openLoopRotation = rotationVal * Constants.Swerve.maxAngularVelocity;

        // When placing or doing a pickup we limit how fast we can drive
        final double limiterRate;
        if (m_placing.getAsBoolean()) {
            limiterRate = 0.4;//0.25
        } else if (m_pickUping.getAsBoolean()) {
            limiterRate = 0.5;//MathUtil.clamp(1.0 - (Timer.getFPGATimestamp() - m_startPickupTime), 0.5, 1.0);
        } else {
            limiterRate = 1.0;
        }
        final var openLoopTranslation = translation.times(Constants.Swerve.maxSpeed).times(limiterRate);
        final Translation2d visionTranslation = new Translation2d(openLoopTranslation.getX(), visionFeedback * Constants.Swerve.maxSpeed);

        /* 
         * When placing make the robot face the direction it started in
         * When picking we use the flipped direction
         * Otherwise we allow normal control of rotation
         */
        final double rotationModifier;
        if (Superstructure.getInstance().doPickupYaw()) {
            rotationModifier = Constants.Generation.pureThetaController.calculate(
                    m_swerve.getYaw().getRadians(),
                    Units.degrees2Rads(0)
            );
        } else if (m_placing.getAsBoolean() || m_doCubeYawLock.getAsBoolean()) {
            rotationModifier = Constants.Generation.pureThetaController.calculate(
                    m_swerve.getYaw().getRadians(),
                    Units.degrees2Rads(180)
            );
        } else {
            rotationModifier = openLoopRotation;
        }

//        // pickup and placement yaw control temporarily disabled
//        final double rotationModifier = openLoopRotation;

        final var doEvasion = !Superstructure.getInstance().doPickupYaw() && !m_placing.getAsBoolean() && m_doEvasionSup.getAsBoolean();

        /* Drive */
        m_swerve.setDrive(
            (DO_VISION_ADJUSTMENT && m_useVision.getAsBoolean()) ? visionTranslation : openLoopTranslation,
            rotationModifier,
            !m_isRobotCentric.getAsBoolean(),
            true,
            doEvasion
        );
    }
}
