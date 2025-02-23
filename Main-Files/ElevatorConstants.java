package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {

    public static final double maxPos = Units.inchesToMeters(52); // Maximum elevator height in inches (adjust based on actual mechanism)
    public static final double countsPerInch = 4096; // Encoder counts per inch (depends on your encoder)

    public static final double kP = 1.5; // idk. Proportional gain (tune for responsiveness)
    public static final double kI = 0; // Integral gain (often kept low or zero)
    public static final double kD = 0; // Derivative gain (tune this based on your control needs)   

    public static final double maxAcceleration = Meters.of(0.2).per(Second).in(MetersPerSecond); // Maximum acceleration in inches per second squared
    public static final double maxVelocity = 256.47;//Meters.of(0.2).per(Second).in(MetersPerSecond); // Maximum velocity in inches per second

    public static final int limitSwitchPort = 0; // Digital input port for the limit switch (check wiring)

    public static final int realElevatorID = 10;  // Motor controller ID (keep or change based on setup)

    public static final double downPos = Units.inchesToMeters(0); // Position when the elevator is fully down
    public static final double L1 = Units.inchesToMeters(1); // Height at L1
    public static final double L2 = Units.inchesToMeters(11); // Height at L2
    public static final double L3 = Units.inchesToMeters(26); // Height at L3
    public static final double L4 = Units.inchesToMeters(52); // Height at L4

    public static final double max_output = 0.5; // Maximum motor power (usually 1.0 for full power)
    public static final double bottomPos = 0.0; // Homing position (same as downPos)
    public static final double posTolerance = Inches.of(2).in(Meters); // Allowable error in inches. used to check if the elevator is within a small tolerance of the target height. change this

    public static final double kS = 0.0; // Static friction feedforward (depends on motor characteristics)
    public static final double kG = 0.52; // Gravity feedforward (compensates for gravity)
    public static final double kV = 6.14; // Velocity feedforward (depends on your system)
    public static final double kA = 0.05; 

    public static final double minPos = 0; // Minimum setpoint (same as downPos)
    public static final double kElevatorDrumRadius = Units.inchesToMeters(3.0);
    public static final double kElevatorGearing = 3.0;
//stall load 49.64
}
