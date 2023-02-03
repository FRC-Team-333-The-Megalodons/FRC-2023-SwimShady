// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /* Device IDs */
    public static class DeviceIDs {

        //Device IDs
        public static final int POWER_DISTRIBUTION_BOARD  = 0;
        public static final int PNEUMATICS_CONTROL_MODULE = 1;

        // Chassis IDs
        public static final int LEFT_LEADER_ID      = 1;
        public static final int LEFT_FOLLOWER1_ID   = 2;
        public static final int LEFT_FOLLOWER2_ID   = 3;

        public static final int RIGHT_LEADER_ID     = 4;
        public static final int RIGHT_FOLLOWER1_ID  = 7;
        public static final int RIGHT_FOLLOWER2_ID  = 8;

        // Intake IDs
        public static final int LEFT_INTAKE_MOTOR_ID  = 9;
        public static final int RIGHT_INTAKE_MOTOR_ID = 8;

        // Elevator IDs
        public static final int ELEVATOR_MOTOR1 = 5;
        public static final int ELEVATOR_MOTOR2 = 6;
        public static final int WORM_GEAR_MOTOR = 7;

        //Sensor IDs
        public static final int MAGNETIC_LIMIT_SWITCH = 0;

        // Joysticks Port IDs
        public static final int JOYSTICK_PORT = 0;

        // Solenoids
        public static final int INTAKE_SOLENOID1         = 0;
        public static final int INTAKE_SOLENOID2         = 2;
        public static final int ELEVATOR_SOLENOID_REAL   = 3;
        public static final int ELEVATOR_SOLENOID_FAKE   = 5;
        public static final int DRIVETRAIN_SOLENOID_LOW  = 1;
        public static final int DRIVETRAIN_SOLENOID_HIGH = 4;
    }

    /* Buttons for Joystick */
    public static class JoystickButtons {
        public static final int LOW_GEAR    = 2;
        public static final int ELEVATE     = 3;
        public static final int DESCALATE   = 4;
        public static final int INTAKE_IN   = 5;
        public static final int INTAKE_OUT  = 6;
    }
}