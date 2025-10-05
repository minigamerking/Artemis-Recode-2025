package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;

import static frc.robot.Constants.RobotConstants.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;


public class SwerveSubsystem extends SubsystemBase {
    public SwerveSubsystem(CommandXboxController controller) {
        System.out.println("Swerve Subsystem initialized");
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public final SwerveModule fl_module = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule fr_module = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffset,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule bl_module = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffset,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule br_module = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffset,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);;

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    public SwerveModuleState[] setChassisSpeeds(ChassisSpeeds desiredSpeed) {
        SwerveModuleState[] newStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(desiredSpeed);

        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        fl_module.setDesiredState(newStates[0]);
        //fr_module.setDesiredState(newStates[1]);
        //bl_module.setDesiredState(newStates[2]);
        //br_module.setDesiredState(newStates[3]);

        return newStates;
    }

    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("Swerve Module States", SwerveModuleState.struct).publish();

    public void drive(double xSpeed, double ySpeed, double rot) {
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
            xSpeed,
            ySpeed,
            rot
        );

        setChassisSpeeds(desiredSpeeds);

        //fl_module.setDesiredState(new SwerveModuleState(xSpeed, Rotation2d.fromDegrees(rot)));

        SwerveModuleState[] loggingStates = {fr_module.getState(), fl_module.getState(), br_module.getState(), bl_module.getState()};
        
        publisher.set(loggingStates);
    }

    @Override
    public void periodic() {
        fl_module.periodic();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getYaw(), chassisLength);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void stopModules() {
        fl_module.stop();
        fr_module.stop();
        bl_module.stop();
        br_module.stop();
    }

    public void resetHeading() {
        gyro.reset();
    }
}
