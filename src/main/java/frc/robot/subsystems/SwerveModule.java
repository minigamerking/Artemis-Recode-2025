package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    private final CANcoder steerEncoder;
    private final RelativeEncoder driveEncoder;

    private final SparkClosedLoopController driverClosedController;
    private final SparkClosedLoopController steerClosedController;

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

        this.driverClosedController = driveMotor.getClosedLoopController();
        this.steerClosedController = steerMotor.getClosedLoopController();

        this.positionInBot = positionInBot;
        this.state = new SwerveModuleState();
    }

    public SwerveModuleState getState() {
        return this.state;
    }

    public void setDesiredState(SwerveModuleState newState) {
        this.state = newState;
        this.state.optimize(new Rotation2d(this.steerEncoder.getAbsolutePosition().getValue()));
    }

    public Translation2d getPosition() {
        return this.positionInBot;
    }
}
