package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;


public class SwerveSubsystem extends SubsystemBase {
    public SwerveSubsystem() {
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
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
        0);

    public final SwerveModule fr_module = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffset,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
        1);

    public final SwerveModule bl_module = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffset,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
        2);

    public final SwerveModule br_module = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffset,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
        3);

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(), new SwerveModulePosition[]{fl_module.getPosition(), fr_module.getPosition(), bl_module.getPosition(), br_module.getPosition()});

    private SwerveModuleState[] desiredStates = {};

    public void setChassisSpeeds(ChassisSpeeds desiredSpeed) {
        SwerveModuleState[] newStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(desiredSpeed);

        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        fl_module.setDesiredState(newStates[0]);
        fr_module.setDesiredState(newStates[1]);
        bl_module.setDesiredState(newStates[2]);
        br_module.setDesiredState(newStates[3]);

        this.desiredStates = newStates;
    }

    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("Swerve Module States", SwerveModuleState.struct).publish();

    public void drive(double xSpeed, double ySpeed, double rot) {
        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            rot,
            getRotation2d()
        );

        setChassisSpeeds(desiredSpeeds);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPose(pose);
    }

    @Override
    public void periodic() {
        fl_module.periodic();
        fr_module.periodic();
        bl_module.periodic();
        br_module.periodic();

        SwerveModuleState[] loggingStates = null;

        if (Robot.isSimulation()) {
            loggingStates = desiredStates;
        } else {
            loggingStates = new SwerveModuleState[]{fl_module.getState(), fr_module.getState(), bl_module.getState(), br_module.getState()};
            // loggingStates = desiredStates;
        }
        
        odometry.update(getRotation2d(), new SwerveModulePosition[]{fl_module.getPosition(), fr_module.getPosition(), bl_module.getPosition(), br_module.getPosition()});
        
        publisher.set(loggingStates);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "PID P", 
            () -> fl_module.getController().getP(),
            (double p) -> this.setP(p));
        builder.addDoubleProperty(
            "PID D", 
            () -> fl_module.getController().getD(),
            (double d) -> this.setD(d));
        builder.addDoubleProperty("FL Module Setpoint", 
            () -> fl_module.getController().getSetpoint(),
            null);
        builder.addDoubleProperty("FR Module Setpoint", 
            () -> fr_module.getController().getSetpoint(),
            null);
        builder.addDoubleProperty("BL Module Setpoint", 
            () -> bl_module.getController().getSetpoint(),
            null);
        builder.addDoubleProperty("BR Module Setpoint", 
            () -> br_module.getController().getSetpoint(),
            null);
        builder.addDoubleProperty("FL Module Heading",
            () -> fl_module.getAbsoluteEncoderDeg(),
             null);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-gyro.getRotation2d().getDegrees());
    }

    public void setP(double p) {
        fl_module.setControllerP(p);
        fr_module.setControllerP(p);
        bl_module.setControllerP(p);
        br_module.setControllerP(p);
    }

    public void setD(double d) {
        fl_module.setControllerD(d);
        fr_module.setControllerD(d);
        bl_module.setControllerD(d);
        br_module.setControllerD(d);
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

    public Command resetSwerveHeadings() {
        return Commands.runOnce(() -> {
            fl_module.resetEncoders();
            fr_module.resetEncoders();
            bl_module.resetEncoders();
            br_module.resetEncoders();
        }).ignoringDisable(true);
    }

    public Command getAbsoluteEncoder() {
        return Commands.runOnce(() -> {
            fl_module.getAbsoluteEncoderDeg();
            fr_module.getAbsoluteEncoderDeg();
            bl_module.getAbsoluteEncoderDeg();
            br_module.getAbsoluteEncoderDeg();
        }).ignoringDisable(true);
    }
}
