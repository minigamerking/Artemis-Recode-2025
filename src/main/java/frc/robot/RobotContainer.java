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

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;



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

    public final RobotContainer.AdvancedCommands commands = new AdvancedCommands();

    public final Autons autons;

    public final Field2d field;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // swerve.setDefaultCommand(new TestDriveCommand(
        //     swerve,
        //     () -> -driverController.getLeftY(),
        //     () -> -driverController.getLeftX()
        // ));

        this.field = new Field2d();

        swerve.setDefaultCommand(new SwerveDriveCommand(swerve,
                () -> driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX())
        );
        // Configure the trigger bindings
        configureBindings();

        this.autons = new Autons(swerve, intake, this);

        SmartDashboard.putData("Shooter", shooter);
        SmartDashboard.putData("Calibrate Arm", arm.commands.calibrateArm());
        SmartDashboard.putData("Intake", intake);
        SmartDashboard.putData("Swerve", swerve);
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

        operatorController.x().whileTrue(this.commands.shoot());

        driverController.rightTrigger().whileTrue(this.commands.intakeUntilReady().alongWith(this.swerve.setSpeedMultiplier(0.3)));
        driverController.leftTrigger().whileTrue(this.commands.outtake());
        
        driverController.rightBumper().onTrue(this.swerve.setSpeedMultiplier(0.5)).onFalse(this.swerve.setSpeedMultiplier(1));

        driverController.start().onTrue(this.swerve.resetHeading());

        operatorController.back().and(operatorController.a()).onTrue(swerve.sysIdQuasistatic(Direction.kForward));
        operatorController.back().and(operatorController.b()).onTrue(swerve.sysIdQuasistatic(Direction.kReverse));
        operatorController.back().and(operatorController.x()).onTrue(swerve.sysIdDynamic(Direction.kForward));
        operatorController.back().and(operatorController.y()).onTrue(swerve.sysIdDynamic(Direction.kReverse));
    }

    public void stop() {
        this.shooter.stop();
        this.intake.stop();
        this.arm.stop();
    }

    public class AdvancedCommands {
        public Command prepareToShoot(double angle) {
            Arm.Commands arm = RobotContainer.this.arm.commands;
            Shooter.Commands shooter = RobotContainer.this.shooter.commands;

            return arm.goToRestingAngle(angle)
                .alongWith(shooter.spinUp(1, 0.05));
        }

        public Command intakeUntilReady() {
            return RobotContainer.this.intake.commands.featherIntake()
                .until(RobotContainer.this.intake::getBeamBreak)
                .andThen(correctNotePosition())
                .handleInterrupt(() -> RobotContainer.this.swerve.setSpeed(1));
        }

        public Command correctNotePosition() {
            Intake.Commands intake = RobotContainer.this.intake.commands;
            Shooter.Commands shooter = RobotContainer.this.shooter.commands;
            BooleanSupplier BeamBreakSensorNotBroken = () ->
                                                            !RobotContainer.this.intake.laserCan.get();
            
            Command firstCorrect = intake.outtake(0.5)
                .alongWith(shooter.shoot(-0.5))
                .until(BeamBreakSensorNotBroken);
            
            Command secondCorrect = intake.intake(0.25)
                .until(RobotContainer.this.intake::getBeamBreak);
            
            Command lastCorrect = intake.outtake(0.1)
                .until(BeamBreakSensorNotBroken);

            Command correct = firstCorrect
                .andThen(secondCorrect)
                .andThen(lastCorrect);

            return correct
                .unless(BeamBreakSensorNotBroken);
        }

        public Command shoot() {
            return this.prepareToShoot(55)
                        .withTimeout(1)
                        .andThen(new WaitCommand(0.5))
                        .andThen(RobotContainer.this.intake.commands.intake())
                        .finallyDo(() -> {
                            RobotContainer.this.shooter.stop();
                            RobotContainer.this.arm.setRestingAngle(0);
                        });
        }

        public Command outtake() {
            return RobotContainer.this.shooter.commands.shoot(-0.5).alongWith(RobotContainer.this.intake.commands.outtake());
        }
    }
}
