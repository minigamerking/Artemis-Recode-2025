package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Intake extends SubsystemBase {
    
    private final SparkMax leftIntake;

    private final SparkMax rightIntake;

    private final LaserCan laserCan;

    public final Intake.Commands commands;


    public Intake(int leftIntakeID, int rightIntakeID, int laserCanID) {
        this.leftIntake = new SparkMax(leftIntakeID, MotorType.kBrushless);

        this.rightIntake = new SparkMax(rightIntakeID, MotorType.kBrushless);

        this.laserCan = new LaserCan(laserCanID);
        
        try {
            this.laserCan.setRangingMode(RangingMode.SHORT);
            this.laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_50MS);
            this.laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(
                8, 
                8, 
                6, 
                6));
        } catch (ConfigurationFailedException exception) {
            System.err.println(exception);
        }

        this.commands = new Commands();
    }


    public void stop() {
        this.leftIntake.stopMotor();
        this.rightIntake.stopMotor();
    }

    private void intake() {
        this.leftIntake.set(ArmConstants.kArmIntakeSpeed);
        this.rightIntake.set(-ArmConstants.kArmIntakeSpeed);
    }

    public boolean getBeamBreak() {
        Measurement measurement = this.laserCan.getMeasurement();

        if (measurement == null) return false;

        return Units.metersToInches(measurement.distance_mm / 100) < 8;
    }

    public class Commands {
        public Command intake() {
            return Intake.this.startEnd(
                Intake.this::intake,
                Intake.this::stop);
        }

        public Command stop() {
            return Intake.this.runOnce(
                Intake.this::stop
            );
        }
    }
}
