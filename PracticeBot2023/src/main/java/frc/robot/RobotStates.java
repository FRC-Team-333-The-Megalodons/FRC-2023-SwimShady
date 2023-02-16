// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class RobotStates {

    public static enum ElevatorState{
        LOW,
        MEDIUM,
        HIGH,
        TRAVERSING_DOWN,
        TRAVERSING_UP,
        MOTORS_STOPPED
    }

    public static enum WristStates{
        RETRACTED,
        AT_90_DEGREES,
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
        MOTORS_STOPPED
    }

    public static enum ChassisStates{
        HIGH_GEAR,
        LOW_GEAR,
        MOVING_FORWARD,
        MOVING_BACK,
        TURNING_L,
        TURNING_R
    }
}
