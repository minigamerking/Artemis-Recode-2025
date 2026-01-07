package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final SparkMax leftShooter;

    private final SparkMax rightShooter;

    public final Shooter.Commands commands;

    public Shooter(int leftShooterID, int rightShooterID) {

        this.leftShooter = new SparkMax(leftShooterID, MotorType.kBrushless);
        this.rightShooter = new SparkMax(rightShooterID, MotorType.kBrushless);

        this.leftShooter.configure(getLeftConfig(), null, null);
        this.rightShooter.configure(getRightConfig(), null, null);


        this.commands = new Commands();
    }

    public void stop() {
        this.leftShooter.stopMotor();
        this.rightShooter.stopMotor();
    }

    public SparkMaxConfig getRightConfig() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast);
        
        return config;
    }

    public SparkMaxConfig getLeftConfig() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast);
        
        return config;
    }
    
    public void shoot(double speed) {
        this.leftShooter.set(-speed);
        this.rightShooter.set(speed);
    }

    public void shoot() {
        this.shoot(1);
    }

    public double[] appliedOutputs() {
        return new double[] {this.leftShooter.getAppliedOutput(), this.rightShooter.getAppliedOutput()};
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleArrayProperty("Shooter Outputs", this::appliedOutputs, null);
    }

    public class Commands {
        public Command shoot(double speed) {
            return Shooter.this.startEnd(
                () -> Shooter.this.shoot(speed), 
                Shooter.this::stop);
        }

        public Command spinUp(double speed, double tolerance) {
            return Shooter.this.startRun(
                () -> Shooter.this.shoot(speed),
                () -> {}).until(() -> (Math.abs(Shooter.this.leftShooter.getAppliedOutput() - speed) <= tolerance) &&
                (Math.abs(Shooter.this.rightShooter.getAppliedOutput() - speed) <= tolerance)).handleInterrupt(Shooter.this::stop);
        }

        public Command stop() {
            return Shooter.this.runOnce(
                Shooter.this::stop
            );
        }
    }
}
