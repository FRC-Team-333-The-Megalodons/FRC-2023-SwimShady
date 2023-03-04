// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class RobotStates {

    /*
     * These robot states will assist our autonomous code and also allow the robot to become smarter during teleop. 
     * With these states we can add failsafes and debug modes
     */

    public static enum ElevatorState{
        ELEVATOR_DOWN_WRIST_IN__HOME,
        ELEVATOR_DOWN_WRIST_TRAVERSING_IN_TO_SAFE,
        ELEVATOR_DOWN_WRIST_OUT_SAFE,
        ELEVATOR_DOWN_WRIST_TRAVERSING_OUT_TO_DOWN,
        ELEVATOR_DOWN_WRIST_DOWN__INTAKE,
        ELEVATOR_TRAVERSING_DOWN_TO_MID,
        ELEVATOR_MID_WRIST_IN,
        ELEVATOR_MID_WRIST_OUT__LOWGOAL_SHOOT,
        ELEVATOR_MID_WRIST_OUT__LOWGOAL_PLACE,
        ELEVATOR_TRAVERSING_MID_TO_HIGH,
        ELEVATOR_HIGH_WRIST_IN,
        ELEVATOR_HIGH_WRIST_OUT__HIGHGOAL_SHOOT,
        ELEVATOR_HIGH_WRIST_OUT__HIGHGOAL_PLACE,
        ELEVATOR_UNKNOWN
    }

    public static enum WristStates{
        RETRACTED,
        FULLY_STRAIGHT,
        ROTATING_OUT,
        ROTATING_IN,
        MOTORS_STOPPED
    }

    public static enum IntakeStates{
        OUT,
        IN,
        MOTORS_RUNNING_F,
        MOTORS_RUNNING_R,
        MOTORS_STOPPED,
        OUT_AND_MOTORS_F,
        OUT_AND_MOTORS_R,
        OUT_AND_MOTORS_S,
    }

    public static enum ChassisStates{
        HIGH_GEAR,
        LOW_GEAR,
        MOVING_FORWARD,
        MOVING_BACK,
        TURNING_L,
        TURNING_R,
        STOPPED
    }
}
