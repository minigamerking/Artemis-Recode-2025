package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devicewrappers.RaptorsLaserCan;

public class Intake extends SubsystemBase {
    
    private final SparkMax leftIntake;

    private final SparkMax rightIntake;

    public final RaptorsLaserCan laserCan;

    public final Intake.Commands commands;


    public Intake(int leftIntakeID, int rightIntakeID, int laserCanID) {
        this.leftIntake = new SparkMax(leftIntakeID, MotorType.kBrushless);

        this.rightIntake = new SparkMax(rightIntakeID, MotorType.kBrushless);

        this.laserCan = new RaptorsLaserCan(laserCanID);

        this.commands = new Commands();
    }


    public void stop() {
        this.leftIntake.stopMotor();
        this.rightIntake.stopMotor();
    }

    private void intake(double speed) {
        this.leftIntake.set(speed);
        this.rightIntake.set(-speed);
    }

    public boolean getBeamBreak() {
        return this.laserCan.get().booleanValue();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Beambreak", this.laserCan::get, null);
    }

    public boolean hasGamepiece() {
        return getBeamBreak();
    }

    public boolean noGamepiece() {
        return !getBeamBreak();
    }

    public class Commands {
        public Command intake() {
            return this.intake(1);
        }

        public Command intake(double speed) {
            return Intake.this.startEnd(
                () -> Intake.this.intake(speed),
                Intake.this::stop);
        }

        public Command outtake() {
            return this.outtake(1);
        }

        public Command outtake(double speed) {
            return Intake.this.startEnd(
                () -> Intake.this.intake(-speed), 
                Intake.this::stop);
        }

        public Command stop() {
            return Intake.this.runOnce(
                Intake.this::stop
            );
        }

        public Command featherIntake() {
            Command startIntaking = this.intake().withTimeout(2);
            Command featherIn = this.intake().withTimeout(0.5);
            Command featherOut = this.outtake().withTimeout(0.25);
            Command feather = featherIn.andThen(featherOut).repeatedly();

            return startIntaking.andThen(feather);
        }
    }
}
