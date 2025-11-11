package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.util.FunctionUtilities;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private SwerveModuleState state;

    private final int swerveID;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, int swerveID) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = Rotation2d.fromDegrees(Preferences.getDouble("Module_" + swerveID + "_Angle", 0)).getRotations();
        absoluteEncoder.getConfigurator().apply(config);

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.configure(getDriveConfig(driveMotorReversed), null, PersistMode.kPersistParameters);
        turningMotor.configure(getTurningConfig(turningMotorReversed), null, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        turningPidController = new PIDController(Preferences.getDouble("Module_" + swerveID + "_P", ModuleConstants.kPTurning), 0, Preferences.getDouble("Module_" + swerveID + "_D", ModuleConstants.kDTurning));
        turningPidController.enableContinuousInput(-180, 180);
        turningPidController.setTolerance(0.2);

        //this.state = new SwerveModuleState(0, Rotation2d.fromDegrees(getAbsoluteEncoderDeg()));

        this.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(getAbsoluteEncoderDeg())));

        this.swerveID = swerveID;
    }

    private SparkMaxConfig getDriveConfig(boolean reversed) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.encoder
            .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        config.inverted(reversed);

        return config;
    }

    private SparkMaxConfig getTurningConfig(boolean reversed) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.encoder
            .positionConversionFactor(360);

        config.inverted(reversed);

        return config;
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition() * ModuleConstants.kTurningMotorGearRatio;
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderDeg() {
        double angle = Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValueAsDouble()).getDegrees();
        if (absoluteEncoderReversed) angle *= -1.0;
        // angle -= Preferences.getDouble("Module_" + swerveID + "_Angle", 0);
    
        return FunctionUtilities.normalizeToRange(angle, -180, 180);
    }

    public void resetEncoders() {
        // System.out.println("1 " + this.swerveID + " " + getAbsoluteEncoderDeg() + "\n");
        driveEncoder.setPosition(0);
        double offset = Rotation2d.fromDegrees(Preferences.getDouble("Module_" + swerveID + "_Angle", 0)).plus(Rotation2d.fromDegrees(getAbsoluteEncoderDeg())).getRotations();
        Preferences.setDouble("Module_" + swerveID + "_Angle", Rotation2d.fromRotations(offset).getDegrees());
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        absoluteEncoder.getConfigurator().apply(config);
        // System.out.println("2" + this.swerveID + " " + getAbsoluteEncoderDeg() + "\n");
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAbsoluteEncoderDeg()));
    }

    public void periodic() {
        //System.out.println(this.swerveID + " Desired Angle: " + this.state.angle.getDegrees());
        driveMotor.set(this.state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderDeg()));
        SmartDashboard.putNumber(this.swerveID + " Module Angle: ", getAbsoluteEncoderDeg());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            desiredState.speedMetersPerSecond = 0;
        }
        desiredState.optimize(Rotation2d.fromDegrees(getAbsoluteEncoderDeg()));
        this.state = desiredState;
        turningPidController.setSetpoint(this.state.angle.getDegrees());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public int getSwerveID() {
        return this.swerveID;
    }

    public PIDController getController() {
        return this.turningPidController;
    }

    public void setControllerP(double p) {
        Preferences.setDouble("Module_" + swerveID + "_P", p);
        this.turningPidController.setP(p);
    }

    public void setControllerD(double d) {
        Preferences.setDouble("Module_" + swerveID + "_D", d);
        this.turningPidController.setD(d);
    }
}