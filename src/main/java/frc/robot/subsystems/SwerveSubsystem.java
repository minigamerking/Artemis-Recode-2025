package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.RobotConstants.*;

import com.studica.frc.AHRS;


public class SwerveSubsystem extends SubsystemBase {
    public SwerveSubsystem(CommandXboxController controller) {
        System.out.println("Swerve Subsystem initialized");
    }

    private final SwerveModule fl_module = new SwerveModule(1,2,9, new Translation2d(chassisLength/2,chassisWidth/2));
    private final SwerveModule fr_module = new SwerveModule(3,4,10, new Translation2d(chassisLength/2,-chassisWidth/2));
    private final SwerveModule bl_module = new SwerveModule(5,6,11, new Translation2d(-chassisLength/2,chassisWidth/2));
    private final SwerveModule br_module = new SwerveModule(7,8,12, new Translation2d(-chassisLength/2,-chassisWidth/2));

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        fl_module.getPosition(),
        fr_module.getPosition(),
        bl_module.getPosition(),
        br_module.getPosition()
    );

    public void setChassisSpeeds(ChassisSpeeds desiredSpeed) {
        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(desiredSpeed);

        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, 5);

        fl_module.setState(newStates[0]);
        fr_module.setState(newStates[1]);
        bl_module.setState(newStates[2]);
        br_module.setState(newStates[3]);
    }

    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("Swerve Module States", SwerveModuleState.struct).publish();

    @Override
    public void periodic() {
        
    }

    public void drive(double xSpeed, double ySpeed, double rot, AHRS gyro) {
        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed * kMetersPerSec,
            ySpeed * kMetersPerSec,
            rot * kRadiansPerSec,
            gyro.getRotation2d()
        );



        setChassisSpeeds(desiredSpeeds);

        SwerveModuleState[] loggingState = new SwerveModuleState[] {
            fl_module.getCurrentState(),
            fr_module.getCurrentState(),
            bl_module.getCurrentState(),
            br_module.getCurrentState()
        };
        publisher.set(loggingState);
    }
}
