package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends CommandBase {
    private final Drivetrain m_drivetrainSubsystem;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.MAX_ACCELERATION);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.MAX_ACCELERATION);
    private final SlewRateLimiter turnLimiter = new SlewRateLimiter(Constants.MAX_ANGULAR_ACCELERATION);

    public static interface Controls {
        double driveX();
        double driveY();
        double driveRotation();
        boolean driveResetYaw();
    }

    private Controls controls;

    public DriveCommand(Drivetrain drivetrainSubsystem, Controls controls) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.controls = controls;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        m_drivetrainSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xLimiter.calculate(controls.driveX()),
                yLimiter.calculate(controls.driveY()),
                turnLimiter.calculate(controls.driveRotation()),
                m_drivetrainSubsystem.getGyroscopeRotation()
            )
        );

    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stopModules();
    }
}
