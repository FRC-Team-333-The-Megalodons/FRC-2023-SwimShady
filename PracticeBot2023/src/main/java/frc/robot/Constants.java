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
 * 
 * Sensors to be used on the robot:
 * 1. Magnetic Limit Switch (for elevator limits) (maybe magnet on intake?)
 * 2. Rev Color Sensor (for game piece detection)
 * 3. NEO Encoders (for elevator position)
 * 4. RGB LEDs (to be used in conjunction with the sensors) (for driver(s) and human player)
 * 5. Limelight (Line ups)
 * 6. Camera (april tag line ups, driver(s) visionh)
 * 
 * Some logic to consider:
 * 1. Elevtor stops moving at the top and bottom when limit switch is engaged
 * 2. If no button is being pressed, the elevator returns to the 'home' position and resets encoders
 * 3. When the elevator is up, reduce drive speed
 * 4. Automatic Mode: smart robot, automated tasks
 * 5. Manual Mode: driver(s) has full controll of every subsystem
 * 6. Wrist should ALWAYS be between -90 & 90 to avoid breaking robot (via encoder or magenetic limit swtich)
 * 7. When the limelight is in scoring postion, turn LEDs green, else be red
 * 8. When yellow button is pressed, LEDs go yellow (vice persa for purple button)
 * 9. When robot is balanced, flicker blue and orange
 * 
 * Button Commands: 
 * (NOTE: the cone & cube node scoring postiton will change based on which game piece the robot detects in its claw)
 * Home Position: elevator down, wrist up (90 degrees)
 * Button 2: Floor Pickup: elevate up, wrist down (~45 degrees), claw out, intake
 * Button #: Double Substation Pickup: elevate up, wrist down (0 degrees), claw out, intake
 * Button #: Bottom Row Shot: wrist down (~0 degrees), outake
 * Button #: Middle Row Shot: elevate up, wrist down (~0 degrees), claw out
 * Button #: High   Row Shot: elevate up, wrist down (~0 degrees), claw out
 */

public final class Constants {

    public static class DriverConstants {
        public static final int DRIVER_JOYSTICK_PORT = 0;

        public static final int SHIFT       = -1;
        public static final int INTAKE      = -1;
        public static final int OUTAKE      = -1;
        public static final int CLAW_OUT    = -1;

        public static final int FLOOR_PICKUP                = -1;
        public static final int DOUBLE_SUBSTATION_PICKUP    = -1;
        public static final int HIGH_ROW_SHOT               = -1;
        public static final int MIDDLE_ROW_SHOT             = -1; 
        public static final int BOTTOM_ROW_SHOT             = -1;
    }

    public static class Chassis {
        public static final double AUTO_UNBALANCE_START_SPEED = 0.1;
        public static final double AUTO_UNBALANCE_INCREMENT = 0.02;
        public static final double AUTO_UNBALANCE_MAX_SPEED = 0.83;
        public static final double AUTO_UNBALANCE_TILT_THRESHOLD = -12;
        public static final double TELEOP_X_SLOWDOWN_DIVISOR = 2.35;
        public static final double TELEOP_Y_SLOWDOWN_DIVISOR = 1.5;

    }

    public static class OperatorConstants {
        public static final int OPERATOR_CONTROLLER_PORT = 0;

        public static final int CONE_LIGHTS = -1;
        public static final int CUBE_LIGHTS = -1;
    }

    public static class Elevator {
        public static final double ELEVATOR_POS_BOTTOM = 0;
        public static final double ELEVATOR_POS_GROUND_INTAKE = 11.2;
        public static final double ELEVATOR_POS_DOWN_SLOWDOWN_POINT = 15;
        public static final double ELEVATOR_POS_LOWEST_POINT_WRIST_CAN_MOVE = 10;
        public static final double ELEVATOR_POS_LOWER_THRESHOLD_WHERE_WRIST_NEEDS_TO_BE_LIMITED = 9;
        public static final double ELEVATOR_POS_LOWEST_POINT_ELEVATOR_CAN_GO_WHILE_WRIST_DOWN = 3.5;
        
        public static final double ELEVATOR_BACKUP_UNSAFE = 70;
        public static final double ELEVATOR_POS_MID = 130;
        public static final double ELEVATOR_UP_SLOWDOWN_POINT = 55;
        public static final double ELEVATOR_POS_TOP = 215;
        public static final double ELEVATOR_UP_ESPEED = .93;
        public static final double ELEVATOR_UP_SLOWDOWN_ESPEED = 0.2;
        public static final double ELEVATOR_DOWN_ESPEED = -.93;
        public static final double ELEVATOR_DOWN_SLOWDOWN_ESPEED = -0.20;
        public static final double ELEVATOR_POS_CUBE = 1.5;
    }

    public static class Wrist {
        public static final double WRIST_MAX = 0.93; // 1.02;
        public static final double WRIST_MIN = 0.62; //0.74;
        public static final double WRIST_MIN_WHEN_ELEVATOR_DOWN = 0.69; // (noice) 0.82;
        public static final double WRIST_STRAIGHT = 0.71; // 0.77;
        public static final double WRIST_GROUND_INTAKE = 0.17; // 0.74;
        public static final double WRIST_APPROX_THRESHOLD = 0.07;
        public static final double WRIST_ENCODER_MULTIPLIER = 20;
        public static final double WRIST_CONE_SCORING_POSITION = .74;
        public static final double WRIST_LIMIT_FOR_ELEVATOR_UP = 0.92;
        
        public static final double WRIST_UP_SPEED = 1.0;
        public static final double WRIST_UP_SLOW_SPEED = -0.37;
        public static final double WRIST_DOWN_SPEED = -1.0;

        public static final double WRIST_POS_UPPER_LIMIT = 0.54;
        public static final double WRIST_POS_GRAVITY_THRESHOLD = 0.45; // TODO: This is a guess, check if makes sense.
        public static final double WRIST_POS_LOWER_LIMIT_WHILE_ELEVATOR_UP = 0.1;
        public static final double WRIST_POS_LOWER_LIMIT_WHILE_ELEVATOR_DOWN = 0.24;
        public static final double WRIST_POS_THRESHOLD_WHERE_ELEVATOR_NEEDS_TO_STOP = 0.3;
        public static final double WRIST_POS_TO_SUBSTATION = .28;
        public static final double WRIST_POS_TO_SCORE = .29;
        public static final double WRIST_POS_TO_MID = .14;
    }

    public static class Intake{
        public static final int INTAKE_SHORT = -10;
        public static final int INTAKE_LONG = -100;
        public static final int OUTAKE = 10;
        public static final double INTAKE_SPEED = -0.6;
        public static final double INTAKE_PASSIVE_SPEED = 0;
        public static final double EJECT_SPEED = 0.4;
        public static final double FAST_EJECT_SPEED = 0.6;
    }

    public static class RobotMap{
        /*
         * Motor IDS
         */
        public static final int PORT_DRIVE_TRAIN_L_LEADER = 4;
        public static final int PORT_DRIVE_TRAIN_L_FOLLOWER1 = 5;
        public static final int PORT_DRIVE_TRAIN_L_FOLLOWER2 = 6;

        public static final int PORT_DRIVE_TRAIN_R_LEADER = 1;
        public static final int PORT_DRIVE_TRAIN_R_FOLLOWER1 = 2;
        public static final int PORT_DRIVE_TRAIN_R_FOLLOWER2 = 3;

        public static final int PORT_ELEVATOR1 = 7;
        public static final int PORT_ELEVATOR2 = 8;

        public static final int PORT_WRIST1 = 9;
        public static final int PORT_WRIST2 = 10;

        public static final int PORT_INTAKE1 = 11;
        public static final int PORT_INTAKE2 = 12;

        /*
         * Solenoid IDS
         */

        public static final int PCM_ID = 22; 

        public static final int SHIFT_HIGH = 0;
        public static final int SHIFT_LOW = 0;
        
        public static final int PORT_INTAKE_SQUEEZE = 2;
        public static final int PORT_INTAKE_UNSQUEEZE = 3;
    }

    public static class Values{
        //values that can be used for autonomous.
        public final static double TICKS_PER_METER = 24;
    }
}
