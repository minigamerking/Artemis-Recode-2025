package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final SparkMax leftShooter;

    private final SparkMax rightShooter;

    public final Shooter.Commands commands;

    public Shooter(int leftShooterID, int rightShooterID) {

        this.leftShooter = new SparkMax(leftShooterID, MotorType.kBrushless);
        this.rightShooter = new SparkMax(rightShooterID, MotorType.kBrushless);

        this.commands = new Commands();
    }

    public void stop() {
        this.leftShooter.stopMotor();
        this.rightShooter.stopMotor();
    }
    
    public void shoot(double speed) {
        this.leftShooter.set(speed);
        this.rightShooter.set(speed);
    }

    public void shoot() {
        this.shoot(1);
    }

    public class Commands {
        public Command shoot() {
            return Shooter.this.startEnd(
                Shooter.this::shoot, 
                Shooter.this::stop);
        }

        public Command spinUp() {
            return Shooter.this.startRun(
                Shooter.this::shoot,
                () -> {}).until(() -> Shooter.this.leftShooter.getAppliedOutput() >= 1 && 
                                        Shooter.this.rightShooter.getAppliedOutput() >= 1);
        }

        public Command stop() {
            return Shooter.this.runOnce(
                Shooter.this::stop
            );
        }
    }
}
