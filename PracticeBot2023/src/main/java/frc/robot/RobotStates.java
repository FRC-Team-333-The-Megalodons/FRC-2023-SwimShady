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
        LOW,
        MEDIUM,
        HIGH,
        TRAVERSING_DOWN_FROM_HIGH,
        TRAVERSING_DOWN_FROM_MID,
        TRAVERSING_UP_FROM_LOW,
        TRAVERSING_HIGH_FROM_MID,
        MOTORS_STOPPED
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
