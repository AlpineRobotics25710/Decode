package org.firstinspires.ftc.teamcode.starterbot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Constants {
    public static double TURN_THROTTLE = 0.8;
    public static double TURTLE = 0.65;
    public static double ZERO = 0.0;

    // The feeder servos run this long when a shot is requested.
    public static long FEED_TIME_MS = 170;
    // How long to delay the servos for before they feed the ball into the launcher in milliseconds
    // Waiting for the launcher to reach the target velocity
    public static long LAUNCH_DELAY_MS = 2500;
    public static long LAUNCH_TIME_MS = 1250;

    public static double FEEDER_POWER = 0.35;

    public static double INTAKE_POWER = 0.9; // Intake motors power

    public static double RAMP_INTAKE_POS = 0.0;
    public static double RAMP_OUTTAKE_POS = 0.4;
    public static final double MAX_RAMP_DEGREES = 270.0;

    public static double BLOCKER_OPEN = 0.1;
    public static double BLOCKER_CLOSED = 0.35;

    public static long SPINUP_TIMEOUT_MS = 5000; // abort if never reaches speed

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    public static double LAUNCHER_FAR_VELOCITY = 1660; // Max velocity when shooting the ball from far away
    public static double LAUNCHER_CLOSE_VELOCITY = 1340; // Max velocity when shooting the ball from close
    public static double LAUNCHER_INTAKE_VELOCITY = -1450; // Velocity to run at when intaking the ball
    public static double LAUNCHER_VELOCITY_TOLERANCE = 20;
    public static double LAUNCHER_VELOCITY_TOLERANCE_RAD = 0.02;
    // public static double LAUNCHER_MIN_VELOCITY = 80; // Commented out because unnecessary (for now)

    // Autonomous Constants
    public static double DRIVE_SPEED = 0.5;
    public static double ROTATE_SPEED = 0.2;
    public static double WHEEL_DIAMETER_MM = 96;
    public static double ENCODER_TICKS_PER_REV = 537.7;
    public static double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    public static double TRACK_WIDTH_MM = 404;

    public static final Pose GOAL_POSE = new Pose(12, 140);

    // Camera Constants
    public static double EXPOSURE_MS = 2;
    public static int GAIN = 300;
}

