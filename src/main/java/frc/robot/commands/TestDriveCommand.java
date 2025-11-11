package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TestDriveCommand extends Command {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier xSpeed;
    private final DoubleSupplier ySpeed;

    public TestDriveCommand(
        SwerveSubsystem swerve,
        DoubleSupplier xSpeed,
        DoubleSupplier ySpeed
    ) {
        this.swerve = swerve;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
    }

    @Override
    public void execute() {
        double x = xSpeed.getAsDouble();
        double y = ySpeed.getAsDouble();

        x = Math.abs(x) > OperatorConstants.kDeadband ? x : 0.0;
        y = Math.abs(y) > OperatorConstants.kDeadband ? y : 0.0;

        double hypotenuse = Math.sqrt(Math.pow(x,2) + Math.pow(y, 2));

        double angle = Math.atan2(y,x);

        System.out.println("Swerve Angle: " + angle);

        swerve.br_module.setDesiredState(new SwerveModuleState(hypotenuse * DriveConstants.kPhysicalMaxSpeedMetersPerSecond, Rotation2d.fromRadians(angle)));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.fl_module.stop();
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
