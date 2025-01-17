package frc.robot;

import edu.wpi.first.math.util.Units;


public class Constants {


    public static final class ElevatorConstants {

        public static final int elevator = 10;
        public static final int shooter = 9;
        public static double kGearRatio = 9.0;
        public static final double kMeterPerRevolution = Units.inchesToMeters((1.888*Math.PI)/kGearRatio);

        public static final double KP = 3.0;
        public static final double KI = 0.000000;
        public static final double KD = 0.0000;
        public static final double KIz = 0.0;
        public static final double KFF = 0.3;
        public static final double MaxOutput = 1.0;
        public static final double kMinOutput = -1.0;
        public static final double MaxRPM = 5700;



    }
}
