package frc.robot;


public class ElevatorConstants {

    public static final double maxPos = 52; // Maximum elevator height in inches (adjust based on actual mechanism)
    public static final double countsPerInch = 4096; // Encoder counts per inch (depends on your encoder)

    public static final double kP =.2; // Proportional gain (tune for responsiveness)
    public static final double kI = 0; // Integral gain (often kept low or zero)
    public static final double kD = 0; // Derivative gain (tune this based on your control needs)   

    public static final double maxAcceleration = 0.3; // Maximum acceleration in inches per second squared
    public static final double maxVelocity = 0.3; // Maximum velocity in inches per second

    public static final int limitSwitchPort = 0; // Digital input port for the limit switch (check wiring)

    public static final int realElevatorID = 10;  // Motor controller ID (keep or change based on setup)

    public static final double downPos = 0; // Position when the elevator is fully down
    public static final double L1 = 1; // Height at L1
    public static final double L2 = 11; // Height at L2
    public static final double L3 = 26; // Height at L3
    public static final double L4 = 52; // Height at L4

    public static final double max_output = 0.7; // Maximum motor power (usually 1.0 for full power)
    public static final double bottomPos = 0; // Homing position (same as downPos)
    public static final double posTolerance = 1; // Allowable error in inches. used to check if the elevator is within a small tolerance of the target height. change this

    public static final double kS = 0.3; // Static friction feedforward (depends on motor characteristics)
    public static final double kG = 0.3; // Gravity feedforward (compensates for gravity)
    public static final double kV = 0.3; // Velocity feedforward (depends on your system)
    public static final double kA = 0.3;

    public static final double minPos = 0; // Minimum setpoint (same as downPos)
    public static final int kElevatorDrumRadius = 3;
    public static final double kElevatorGearing = 3;
    

}