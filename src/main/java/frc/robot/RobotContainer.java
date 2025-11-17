// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
   
    
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final CommandXboxController operatorController =
        new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
    
    private final SwerveSubsystem swerve = new SwerveSubsystem();

    private final Arm arm = new Arm(
        ArmConstants.kArmLeftMotorPort,
        ArmConstants.kArmRightMotorPort);

    private final Intake intake = new Intake(
        ArmConstants.kArmLeftIntakePort,
        ArmConstants.kArmRightIntakePort,
        ArmConstants.kShooterBeamBreakPort);

    private final Shooter shooter = new Shooter(
        ArmConstants.kArmLeftShooterPort, 
        ArmConstants.kArmRightShooterPort);

    private final RobotContainer.Commands commands = new Commands();
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // swerve.setDefaultCommand(new TestDriveCommand(
        //     swerve,
        //     () -> -driverController.getLeftY(),
        //     () -> -driverController.getLeftX()
        // ));

        swerve.setDefaultCommand(new SwerveDriveCommand(swerve,
                () -> -driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> driverController.getRightX())
        );
        // Configure the trigger bindings
        configureBindings();

        SmartDashboard.putData("Reset Swerve Headings", swerve.resetSwerveHeadings());
        SmartDashboard.putData("Swerve PID", swerve);
        SmartDashboard.putData("Arm", arm);
    }

    public void periodic() {
        SmartDashboard.putBoolean("Beambreak", this.intake.getBeamBreak());
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        driverController.a().onTrue(swerve.resetSwerveHeadings());

        driverController.y().onTrue(arm.commands.calibrateArm());

        //operatorController.x().whileTrue(arm.commands.shoot());

        driverController.leftTrigger().whileTrue(commands.intakeUntilReady());
    }

    public void stop() {
        this.shooter.stop();
        this.intake.stop();
    }

    public class Commands {
        public Command intakeUntilReady() {
            return RobotContainer.this.intake.commands.intake().until(RobotContainer.this.intake::getBeamBreak);
        }

        public Command shoot() {
            return RobotContainer.this.shooter.commands.spinUp().andThen(RobotContainer.this.intake.commands.intake()).handleInterrupt(RobotContainer.this::stop);
        }
    }
}
