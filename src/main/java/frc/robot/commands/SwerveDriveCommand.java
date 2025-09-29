package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveCommand extends Command {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier xSpeed;
    private final DoubleSupplier ySpeed;
    private final DoubleSupplier rot;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveDriveCommand(
        SwerveSubsystem swerve,
        DoubleSupplier xSpeed,
        DoubleSupplier ySpeed,
        DoubleSupplier rot
    ) {
        this.swerve = swerve;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rot = rot;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    }

    @Override
    public void execute() {
        double x = xSpeed.getAsDouble();
        double y = ySpeed.getAsDouble();
        double rotation = rot.getAsDouble();

        x = Math.abs(x) > OperatorConstants.kDeadband ? x : 0.0;
        y = Math.abs(y) > OperatorConstants.kDeadband ? y : 0.0;
        rotation = Math.abs(rotation) > OperatorConstants.kDeadband ? rotation : 0.0;

        // 3. Make the driving smoother
        x = xLimiter.calculate(x) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        y = yLimiter.calculate(y) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        rotation = turningLimiter.calculate(rotation)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;


        swerve.drive(x, y, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return Set.of(swerve);
    }
}
