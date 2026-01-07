package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

public class Autons {


    private final AutoFactory autoFactory;

    private final Intake intake;

    private final RobotContainer robotContainer;

    public Autons(SwerveSubsystem swerve, Intake intake, RobotContainer container) {
        this.autoFactory = new AutoFactory(
                                            swerve::getPose,
                                            swerve::resetOdometry,
                                            swerve::followTrajectory, 
                                            true, 
                                            swerve);
        
        this.intake = intake;
        this.robotContainer = container;
    }

    public AutoRoutine oneCoralAuto() {
        AutoRoutine routine = autoFactory.newRoutine("oneCoralRoutine");

        AutoTrajectory oneCoral = routine.trajectory("OneCoral");

        routine.active().onTrue(
            Commands.sequence(
                Commands.print("Starting One Coral Auton"),
                oneCoral.resetOdometry(),
                oneCoral.cmd()
            )
        );
        
        return routine;
    }

    public AutoRoutine testAuton() {
        AutoRoutine routine = autoFactory.newRoutine("testAuton");

        AutoTrajectory test = routine.trajectory("TestPath");

        routine.active().onTrue(
            Commands.sequence(
                Commands.print("Starting Test Auton"),
                test.resetOdometry(),
                test.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine returnOfTheKing() {
        AutoRoutine routine = autoFactory.newRoutine("returnOfTheKing");

        AutoTrajectory startToS2 = routine.trajectory("MidToS2");
        AutoTrajectory S2ToStart = routine.trajectory("S2ToMid");
        AutoTrajectory StartToM2 = routine.trajectory("MidToM2");
        AutoTrajectory M2ToStart = routine.trajectory("M2ToMid");
        AutoTrajectory M2ToM1 = routine.trajectory("M2ToM1");
        AutoTrajectory M1ToStart = routine.trajectory("M1ToMid");

        routine.active().onTrue(
            Commands.sequence(
                startToS2.resetOdometry(),
                robotContainer.commands.shoot(),
                startToS2.cmd()
            )
        );

        startToS2.active().whileTrue(robotContainer.commands.intakeUntilReady());
        startToS2.done().onTrue(robotContainer.commands.intakeUntilReady().andThen(S2ToStart.cmd()));

        S2ToStart.done().onTrue(robotContainer.commands.shoot().andThen(StartToM2.cmd()));

        routine.anyActive(StartToM2, M2ToM1).whileTrue(robotContainer.commands.intakeUntilReady());

        Trigger atM2 = StartToM2.done();
        atM2.and(intake::hasGamepiece).onTrue(M2ToStart.cmd());
        atM2.and(intake::noGamepiece).onTrue(M2ToM1.cmd().andThen(M1ToStart.cmd()));
        
        routine.anyDone(M1ToStart, M2ToStart).onTrue(robotContainer.commands.shoot());

        return routine;
    }
}
