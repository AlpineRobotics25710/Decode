package org.firstinspires.ftc.teamcode.starterbot;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {
    public static double ZERO = 0.0;

    // The feeder servos run this long when a shot is requested.
    public static long FEED_TIME_MS = 77; // TODO: Needs to be tuned
    // How long to delay the servos for before they feed the ball into the launcher in milliseconds
    // Waiting for the launcher to reach the target velocity
    public static long LAUNCH_DELAY_MS = 2500; // TODO: Needs to be tuned
    public static long LAUNCH_TIME_MS = 500; // TODO: Needs to be tuned

    public static double FEEDER_POWER = 1.0;

    public static double INTAKE_POWER = 0.85; // Intake motors power

    public static double RAMP_INTAKE_POS = 0.05;
    public static double RAMP_OUTTAKE_POS = 0.475;

    public static double BLOCKER_OPEN = 0.0;
    public static double BLOCKER_CLOSED = 0.36;

    public static long SPINUP_TIMEOUT_MS = 5000; // abort if never reaches speed

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    public static double LAUNCHER_MAX_VELOCITY = 1950; // Max velocity when shooting the ball
    public static double LAUNCHER_INTAKE_VELOCITY = -1750; // Velocity to run at when intaking the ball
    public static double CONTINUE_LAUNCH_SEQUENCE = -1; // Constant to continue the launch sequence
    public static double LAUNCHER_VELOCITY_TOLERANCE = 100;
    // public static double LAUNCHER_MIN_VELOCITY = 80; // Commented out because unnecessary (for now)

    // Autonomous Constants
    public static double DRIVE_SPEED = 0.5;
    public static double ROTATE_SPEED = 0.2;
    public static double WHEEL_DIAMETER_MM = 96;
    public static double ENCODER_TICKS_PER_REV = 537.7;
    public static double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    public static double TRACK_WIDTH_MM = 404;
}

enum RampState {
    // Add more states if necessary
    INTAKE,
    OUTTAKE
}

enum BlockerState {
    OPEN,
    CLOSED
}

enum LaunchSequenceState {
    IDLE,
    SPINNING_UP,
    FEEDING,
    SHOOTING
}
