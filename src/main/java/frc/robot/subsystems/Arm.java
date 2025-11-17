package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.FunctionUtilities;

public class Arm extends SubsystemBase{

    private final ArmFeedforward feedForward;

    private final SparkMax leftMotor;

    private final SparkMax rightMotor;

    private final SparkAbsoluteEncoder leftAbsoluteEncoder;

    private final SparkAbsoluteEncoder rightAbsoluteEncoder;

    public double restingAngle;

    public Arm.Commands commands;

    private PIDController armAnglePIDController;

    public Arm(int leftMotorID, int rightMotorID) {
        feedForward = new ArmFeedforward(
            1.575,
            2.8396,
            0
        );

        commands = new Commands();

        this.leftMotor = new SparkMax(leftMotorID, MotorType.kBrushless);

        this.rightMotor = new SparkMax(rightMotorID, MotorType.kBrushless);

        this.leftAbsoluteEncoder = this.leftMotor.getAbsoluteEncoder();

        this.rightAbsoluteEncoder = this.rightMotor.getAbsoluteEncoder();

        this.rightMotor.configure(getRightMotorConfig(), null, null);
        this.leftMotor.configure(getLeftMotorConfig(), null, null);

        this.armAnglePIDController = new PIDController(0.2, 0, 0);
        

        armAnglePIDController.setSetpoint(ArmConstants.kArmMinAngleDegrees);
    }

    private SparkMaxConfig getRightMotorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(true);

        config.idleMode(IdleMode.kBrake);

        return config;
    }

    private SparkMaxConfig getLeftMotorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(false);

        config.idleMode(IdleMode.kBrake);

        return config;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Arm Angle", this::getAngle, null);
    }

    public void calibrateArm() {
        // Calculate the new zero offset relative to the existing one.
        double newLZeroOffset = Preferences.getDouble("Left_Encoder_Zero", 0) + this.leftAbsoluteEncoder.getPosition();
        
        // Subtract the angle at which the arm is calibrated.
        newLZeroOffset = newLZeroOffset - ArmConstants.kArmCalibrationAngleDegrees;
        
        // Normalize the new zero offset to the range of the encoder.
        newLZeroOffset = FunctionUtilities.normalizeToRange(
                newLZeroOffset,
                0,
                1
        );

        // Set the preference to the new zero
        Preferences.setDouble("Left_Encoder_Zero", newLZeroOffset);

        // Do this for the right encoder
        double newRZeroOffset = Preferences.getDouble("Right_Encoder_Zero", 0) + this.leftAbsoluteEncoder.getPosition();
        
        newRZeroOffset = newRZeroOffset - ArmConstants.kArmCalibrationAngleDegrees;
        
        newRZeroOffset = FunctionUtilities.normalizeToRange(
                newRZeroOffset,
                0,
                1
        );

        Preferences.setDouble("Right_Encoder_Zero", newRZeroOffset);
    }

    public double getEncoderPositionSum() {
        double leftPosition = this.leftAbsoluteEncoder.getPosition() - Preferences.getDouble("Left_Encoder_Zero", 0);
        double rightPosition = this.rightAbsoluteEncoder.getPosition() - Preferences.getDouble("Right_Encoder_Zero", 0);

        return leftPosition + rightPosition;
    }

    public double getAngle() {
        return Rotation2d.fromRotations(
            getEncoderPositionSum() / 2
        ).getDegrees();
    }

    public void stop() {
        this.leftMotor.stopMotor();
        this.rightMotor.stopMotor();
    }

    public void setRestingAngle(double angle) {
        this.restingAngle = FunctionUtilities.applyClamp(
            angle,
            ArmConstants.kArmMinAngleDegrees,
            ArmConstants.kArmMaxAngleDegrees
        );
    }

    @Override
    public void periodic() {
        double currentAngle = this.getAngle();
        double output = armAnglePIDController.calculate(currentAngle);

        boolean wouldOverrun = (
			(output > 0 && currentAngle > ArmConstants.kArmMaxAngleDegrees) ||
			(output < 0 && currentAngle < ArmConstants.kArmMinAngleDegrees)
		);

        double effectiveOutput = wouldOverrun ? 0 : output;

        //this.leftMotor.setVoltage(effectiveOutput);
        //this.rightMotor.setVoltage(effectiveOutput);
    }

    public void rotateToAngle(double angle) {
        double clampedAngle = FunctionUtilities.applyClamp(
            this.getAngle(), 
            ArmConstants.kArmMinAngleDegrees, 
            ArmConstants.kArmMaxAngleDegrees);

        this.armAnglePIDController.setSetpoint(clampedAngle);
    }

    public boolean isAtSetpoint() {
        return this.armAnglePIDController.atSetpoint();
    }

    public class Commands {
        public Command holdAtAngle(double angle) {
            return Arm.this.startEnd(
                () -> {Arm.this.rotateToAngle(angle);},
                () -> {}
                ).handleInterrupt(Arm.this::stop);
        }

        public Command goToAngle(double angle) {
            return this.holdAtAngle(angle)
                .until(Arm.this::isAtSetpoint);
        }

        public Command holdAtRestingAngle() {
            return Arm.this.startEnd(
                () -> {Arm.this.rotateToAngle(Arm.this.restingAngle);},
                () -> {}
                ).handleInterrupt(Arm.this::stop);
        }

        public Command calibrateArm() {
            return Arm.this.runOnce(Arm.this::calibrateArm)
            .withName("Calibrate Arm")
            .ignoringDisable(true);
        }
    }
}
