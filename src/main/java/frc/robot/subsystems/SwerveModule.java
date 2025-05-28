package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    private final CANcoder steerEncoder;
    private final RelativeEncoder driveEncoder;

    private final SimpleMotorFeedforward steerFeedforward;

    private final PIDController steerController;
    private final PIDController driveController;

    private SwerveModuleState state;

    private final Translation2d positionInBot;

    public SwerveModule(
            int driveID,
            int steerID,
            int encoderID,
            Translation2d positionInBot
    ) {
        this.driveMotor = new SparkMax(driveID, SparkLowLevel.MotorType.kBrushless);
        this.steerMotor = new SparkMax(steerID, SparkLowLevel.MotorType.kBrushless);

        this.steerEncoder = new CANcoder(encoderID);
        this.driveEncoder = driveMotor.getEncoder();

        this.steerFeedforward = new SimpleMotorFeedforward(0.5, 0.005);
        this.steerController = new PIDController(0.04, 0, 0);

        this.steerController.setTolerance(2);
        this.steerController.setSetpoint(0);
        this.steerController.enableContinuousInput(-180, 180);


        this.driveController = new PIDController(0.1, 0, 0);

        this.positionInBot = positionInBot;
        this.state = new SwerveModuleState();


    }

    public SwerveModuleState getCurrentState() {
        return this.state;
    }

    public void setState(SwerveModuleState newState) {
        newState.optimize(new Rotation2d(getSteeringHeading()));

        System.out.println(newState.angle.getDegrees());

        driveMotor.set(driveController.calculate(newState.speedMetersPerSecond));
        steerMotor.set(steerController.calculate(newState.angle.getDegrees()));
        this.state = new SwerveModuleState(driveMotor.get(), new Rotation2d(getSteeringHeading()));
    }

    public Translation2d getPosition() {
        return this.positionInBot;
    }

    public Angle getSteeringHeading() {
        return Degrees.of(
            normalizeRange(
                this.steerEncoder.getPosition().getValueAsDouble(), 
                180, 
                -180)
        );
    }

    private double normalizeRange(double input, double maximum, double minimum) {
        double delta = maximum - minimum;
        
        return minimum + ((input - minimum + delta) % delta);
    }
}
