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

    public static class OperatorConstants {
        public static final int OPERATOR_CONTROLLER_PORT = 0;

        public static final int CONE_LIGHTS = -1;
        public static final int CUBE_LIGHTS = -1;
    }

    public static class RobotMap{
        /*
         * Motor IDS
         */
        public static final int DRIVE_TRAIN_L_LEADER = 4;
        public static final int DRIVE_TRAIN_L_FOLLOWER1 = 5;
        public static final int DRIVE_TRAIN_L_FOLLOWER2 = 6;

        public static final int DRIVE_TRAIN_R_LEADER = 1;
        public static final int DRIVE_TRAIN_R_FOLLOWER1 = 2;
        public static final int DRIVE_TRAIN_R_FOLLOWER2 = 3;

        public static final int ELEVATOR1 = 7;
        public static final int ELEVATOR2 = 8;

        public static final int WRIST1 = 9;
        public static final int WRIST2 = 10;

        public static final int INTAKE1 = 11;
        public static final int INTAKE2 = 14;

        /*
         * Solenoid IDS
         */

        public static final int PCM_ID = 1; 

        public static final int SHIFT_HIGH = 0;
        public static final int SHIFT_LOW = 0;
        
        public static final int INTAKE_SQUEEZE = 0;
        public static final int INTAKE_UNSQUEEZE = 0;
    }

    public static class Values{
        //values that can be used for autonomous.
        public final static double TICKS_PER_METER = 0;
    }
}
