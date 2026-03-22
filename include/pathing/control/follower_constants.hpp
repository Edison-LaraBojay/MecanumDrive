#pragma once

namespace pathing::control {

struct FollowerConstants {

    /*
     * LOOKAHEAD
     * Distance ahead on the path the robot targets.
     * Larger = smoother but cuts corners more.
     * Smaller = tighter tracking but more oscillation risk.
     * this is like how tightly we wanna stick to the actual curve vs approximation
     */
    static constexpr double LOOKAHEAD_DISTANCE = 6.0;


    /*
     * POSITION TOLERANCE
     * robot at final point like exit condition type shi
     */
    static constexpr double POSITION_TOLERANCE = 0.5;


    /*
     * HEADING TOLERANCE (radians)
     */
    static constexpr double HEADING_TOLERANCE = 0.05;


    /*
     * MAX VELOCITIES
     * Used to clamp controller output.
     */
    static constexpr double MAX_VELOCITY = 1.0;
    static constexpr double MAX_ACCELERATION = 1.0;
    static constexpr double MAX_ANGULAR_VELOCITY = 3.0;


    /*
     * FEEDFORWARD TERMS
     */
    static constexpr double kV = 1.0;
    static constexpr double kA = 0.0;


    /*
     * PID DEFAULTS
     * These can be overridden elsewhere if needed.
     */

    // forward (tangent direction)
    static constexpr double FORWARD_kP = 1.0;
    static constexpr double FORWARD_kI = 0.0;
    static constexpr double FORWARD_kD = 0.0;

    // lateral (cross-track)
    static constexpr double LATERAL_kP = 1.0;
    static constexpr double LATERAL_kI = 0.0;
    static constexpr double LATERAL_kD = 0.0;

    // heading
    static constexpr double HEADING_kP = 2.0;
    static constexpr double HEADING_kI = 0.0;
    static constexpr double HEADING_kD = 0.0;


    /*
     * STOPPING BEHAVIOR
     */
    static constexpr double STOP_VELOCITY_THRESHOLD = 0.05;
    static constexpr double STOP_ANGULAR_THRESHOLD = 0.05;

};

}