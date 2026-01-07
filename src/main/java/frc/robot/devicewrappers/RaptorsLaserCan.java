package frc.robot.devicewrappers;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

import java.util.function.Supplier;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

public class RaptorsLaserCan implements Supplier<Boolean>{

    protected LaserCan laserCan;


    public RaptorsLaserCan(int laserCanID) {
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
    }

    @Override
    public Boolean get() {
        Measurement measurement = laserCan.getMeasurement();

        if (measurement == null) return false;

        return Millimeters.of(measurement.distance_mm).in(Inches) < 8;
    }
    
}
