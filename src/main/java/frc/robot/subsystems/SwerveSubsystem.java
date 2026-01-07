package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.FunctionUtilities;

import static edu.wpi.first.units.Units.Volts;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;


public class SwerveSubsystem extends SubsystemBase {

    public final SwerveModule fl_module = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
        0);

    public final SwerveModule fr_module = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
        1);

    public final SwerveModule bl_module = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
        2);

    public final SwerveModule br_module = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
        3);

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
                                                                            getRotation2d(), 
                                                                            getPositions());

    private SwerveModuleState[] desiredStates = {};

    private double speedMultiplier = 1;

    private double maxSpeedMultiplier = 1;

    private PIDController xController = new PIDController(1, 0, 0);
    private PIDController yController = new PIDController(1, 0, 0);
    private PIDController headingController = new PIDController(3.5, 0, 0);

    private SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (voltage) -> {this.runVolts(voltage.in(Volts));}, 
            null, 
            this
        )
    );

    private final Field2d field;

    public SwerveSubsystem() {
        System.out.println("Swerve Subsystem initialized");
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetGyro();
                resetOdometry(new Pose2d(new Translation2d(), getRotation2d()));
            } catch (Exception e) {
            }
        }).start();

        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.field = new Field2d();

        SmartDashboard.putData("Field", field);
    }

    public void followTrajectory(SwerveSample sample) {
        System.out.println("Following Trajectory...");
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        // Apply the generated speeds
        setChassisSpeeds(speeds, false);
    }

    public void setChassisSpeeds(ChassisSpeeds desiredSpeed, boolean fieldRelative) {
        System.out.println(fieldRelative);
        //ChassisSpeeds speeds = (fieldRelative) ? ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeed, getRotation2d()) : desiredSpeed;
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeed, getRotation2d());


        SwerveModuleState[] newStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        fl_module.setDesiredState(newStates[0]);
        fr_module.setDesiredState(newStates[1]);
        bl_module.setDesiredState(newStates[2]);
        br_module.setDesiredState(newStates[3]);

        this.desiredStates = newStates;
    }

    private void runVolts(double volts) {
        fl_module.runVolts(volts/2);
        fr_module.runVolts(-volts/2);
        bl_module.runVolts(volts/2);
        br_module.runVolts(-volts/2);
    }

    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("Swerve Module States", SwerveModuleState.struct).publish();

    public void drive(double xSpeed, double ySpeed, double rot) {
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
            xSpeed,
            ySpeed,
            rot
        );

        setChassisSpeeds(desiredSpeeds.times(this.speedMultiplier), true);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getPositions(), pose);
    }

    private SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        positions[0] = fl_module.getPosition();
        positions[1] = fr_module.getPosition();
        positions[2] = bl_module.getPosition();
        positions[3] = br_module.getPosition();

        return positions;
    }

    public void setSpeed(double mult) {

        this.speedMultiplier = FunctionUtilities.applyClamp(
            mult, 
            0, 
            this.maxSpeedMultiplier);
    }

    @Override
    public void periodic() {
        fl_module.periodic();
        fr_module.periodic();
        bl_module.periodic();
        br_module.periodic();

        odometry.update(getRotation2d(), getPositions());

        this.field.setRobotPose(getPose());

        SwerveModuleState[] loggingStates = null;

        if (Robot.isSimulation()) {
            loggingStates = desiredStates;
        } else {
            loggingStates = new SwerveModuleState[]{fl_module.getState(), fr_module.getState(), bl_module.getState(), br_module.getState()};
            // loggingStates = desiredStates;
        }
        
        
        
        publisher.set(loggingStates);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
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
        builder.addBooleanProperty("New Driver Mode", 
            () -> this.maxSpeedMultiplier < 1 ? true : false,
            (boolean mult) -> this.setMaxSpeed(mult ? 0.5 : 1));
        builder.addDoubleProperty("Gyro", 
            () -> getRotation2d().getDegrees(),
            null);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(gyro.getRotation2d().getDegrees());
    }

    private void setMaxSpeed(double mult) {
        System.out.println(mult);
        this.maxSpeedMultiplier = mult;
        this.setSpeed(mult);
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

    public void resetGyro() {
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

    public Command resetHeading() {
        return Commands.runOnce(this::resetGyro);
    }

    public Command setSpeedMultiplier(double mult) {
        return Commands.runOnce(() -> this.setSpeed(mult));
    }

    public Command sysIdQuasistatic(Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
