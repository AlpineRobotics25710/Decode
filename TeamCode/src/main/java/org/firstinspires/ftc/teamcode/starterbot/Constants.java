package org.firstinspires.ftc.teamcode.starterbot;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {
    // The feeder servos run this long when a shot is requested.
    public static long FEED_TIME_MS = 500; // TODO: Needs to be tuned
    // How long to delay the servos for before they feed the ball into the launcher in milliseconds
    // Waiting for the launcher to reach the target velocity
    public static long LAUNCH_DELAY_MS = 2500; // TODO: Needs to be tuned
    public static long LAUNCH_TIME_MS = 1500; // TODO: Needs to be tuned

    public static double STOP_POWER = 0.0; //We send this power to the servos when we want them to stop.
    public static double FULL_POWER = 1.0;

    public static double INTAKE_POWER = 0.85; // Intake motors power

    public static double RAMP_INTAKE_POS = 0.05;
    public static double RAMP_OUTTAKE_POS = 0.475;

    public static double BLOCKER_OPEN = 0.0;
    public static double BLOCKER_CLOSED = 0.36;

    public static double LAUNCH_READY_TOLERANCE = 0.05; // ±5% of target velocity OK
    public static long   READY_HOLD_MS          = 150;  // must hold at-speed this long
    public static long   SPINUP_TIMEOUT_MS      = 4000; // abort if never reaches speed

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    public static double LAUNCHER_MAX_VELOCITY = 230; // Max velocity when shooting the ball
    public static double LAUNCHER_INTAKE_VELOCITY = -70; // Velocity to run at when intaking the ball
    public static double CONTINUE_LAUNCH_SEQUENCE = -1; // Constant to continue the launch sequence
    // public static double LAUNCHER_MIN_VELOCITY = 80; // Commented out because unnecessary (for now)
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
