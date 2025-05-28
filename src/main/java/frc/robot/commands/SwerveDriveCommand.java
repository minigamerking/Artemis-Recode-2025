package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveCommand extends Command {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier xSpeed;
    private final DoubleSupplier ySpeed;
    private final DoubleSupplier rot;
    private final AHRS gyro;

    public SwerveDriveCommand(
        SwerveSubsystem swerve,
        DoubleSupplier xSpeed,
        DoubleSupplier ySpeed,
        DoubleSupplier rot,
        AHRS gyro
    ) {
        this.swerve = swerve;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rot = rot;
        this.gyro = gyro;
    }

    @Override
    public void execute() {
        swerve.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble(), gyro);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, gyro); // Stop the robot when the command ends
    }


    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return Set.of(swerve);
    }
}
