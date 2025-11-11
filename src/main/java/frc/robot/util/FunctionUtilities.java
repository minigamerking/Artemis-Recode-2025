package frc.robot.util;

public class FunctionUtilities {
    public static double normalizeToRange(double input, double minimum, double maximum) {
        double delta = maximum - minimum;
        return minimum + ((input - minimum + delta) % delta);
    }

    public static double applyClamp(double input, double minimum, double maximum) {
        if (input < minimum) return minimum;
        else if (input > maximum) return maximum;
        else return input;
    }
}
