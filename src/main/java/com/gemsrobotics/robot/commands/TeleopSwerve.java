package com.gemsrobotics.robot.commands;

import com.gemsrobotics.robot.Constants;
import com.gemsrobotics.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve m_swerve;
    private DoubleSupplier m_translation;
    private DoubleSupplier m_strafe;
    private DoubleSupplier m_rotation;
    private BooleanSupplier m_isRobotCentric;
    private SlewRateLimiter m_translationFilter;
    private SlewRateLimiter m_strafeFilter;
    private SlewRateLimiter m_rotationFilter;

    public TeleopSwerve(
            final Swerve swerve,
            final DoubleSupplier translationSup,
            final DoubleSupplier strafeSup,
            final DoubleSupplier rotationSup,
            final BooleanSupplier robotCentricSup
    ) {
        m_swerve = swerve;
        addRequirements(swerve);

        m_translation = translationSup;
        m_strafe = strafeSup;
        m_rotation = rotationSup;
        m_isRobotCentric = robotCentricSup;
        m_translationFilter = new SlewRateLimiter(3);
        m_strafeFilter = new SlewRateLimiter(3);
        m_rotationFilter = new SlewRateLimiter(3);
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
        
        /* Drive */
        m_swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !m_isRobotCentric.getAsBoolean(),
            true
        );
    }
}
