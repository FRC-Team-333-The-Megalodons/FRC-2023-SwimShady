// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotStates;
import frc.robot.RobotStates.ChassisStates;
import frc.robot.RobotStates.ElevatorState;
import frc.robot.utils.Containers.*;

public class Chassis extends SubsystemBase {
  Multi_CANSparkMax m_leftSparks, m_rightSparks;
  Multi_RelativeEncoder m_leftEncoders, m_rightEncoders;
  MotorControllerGroup m_leftMotors, m_rightMotors;

  Joystick m_stick;

  DifferentialDrive m_drive;

  RobotStates.ChassisStates m_chassisState;
  PneumaticHub m_hub;
  DoubleSolenoid m_solenoid;

  ElevatorState m_elevatorState;

  public Chassis(PneumaticHub hub) {
    m_leftSparks = new Multi_CANSparkMax(
        new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_L_LEADER, MotorType.kBrushless),
        new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_L_FOLLOWER1, MotorType.kBrushless),
        new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_L_FOLLOWER2, MotorType.kBrushless)
    );
    m_rightSparks = new Multi_CANSparkMax(
        new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_R_LEADER, MotorType.kBrushless),
        new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_R_FOLLOWER1, MotorType.kBrushless),
        new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_R_FOLLOWER2, MotorType.kBrushless)
    );

    m_leftEncoders = m_leftSparks.getMultiEncoder();
    m_rightEncoders = m_rightSparks.getMultiEncoder();

    m_leftMotors = m_leftSparks.getMotorControllerGroup();
    m_rightMotors = m_rightSparks.getMotorControllerGroup();

    m_stick = new Joystick(0);
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
    this.m_hub = hub;
    m_solenoid = hub.makeDoubleSolenoid(0, 1);
  }

  public void setBreak()
  {
    setIdleMode(IdleMode.kBrake);
  }

  public void setCoast()
  {
    setIdleMode(IdleMode.kCoast);
  }

  private void setIdleMode(IdleMode idleMode)
  {
    m_leftSparks.setIdleMode(idleMode);
    m_rightSparks.setIdleMode(idleMode);
  }

  public double getEncodersAverage(){
    return (-m_leftEncoders.getAveragePosition() +
            m_rightEncoders.getAveragePosition()
           ) / 2;
  }

  public void resetEncoders(){
    m_leftEncoders.setPosition(0);
    m_rightEncoders.setPosition(0);
  }

  public double getChassisMetersMoved(){
    return getEncodersAverage()/Constants.Values.TICKS_PER_METER;
  }

  public void arcadeDrive(double x, double y)
  {
    m_drive.arcadeDrive(x, y);
  }

  public void teleopPeriodic(ElevatorState state)
  {
    double x = m_stick.getX(), y = -m_stick.getY();

    if(RobotContainer.TWO_DRIVER_MODE){
      if(m_stick.getTrigger()){//slows down the chassis for lining up
        x /= 1.7;
        y /= 2.5;
        setBreak();
      }else{
        setCoast();
      }
      if(m_stick.getRawButton(2)){
        m_solenoid.set(Value.kForward);
        setBreak();
      }else{
        m_solenoid.set(Value.kReverse);
        setCoast();
      }
    }else{
      if(m_stick.getRawButton(7)){
        x /= 1.7;
        y /= 2.5;
      }
    }
    
    arcadeDrive(x, y);
  }

  // The 'periodic' function is called constantly, even when the robot is not enabled.
  // Therefore, the periodic function should not attempt to move any motors or parts.
  // Instead, it should be used to report data to the SmartDashboard, and to update
  // any Global Variables related to State.
  @Override
  public void periodic() {
    m_chassisState = evaluateState();
    SmartDashboard.putString("chassis State", m_chassisState+"");
    SmartDashboard.putNumber("left chassis encoder", m_leftEncoders.getAveragePosition());
    SmartDashboard.putNumber("right chassis encoder", m_rightEncoders.getAveragePosition());
    SmartDashboard.putNumber("Meters moved", getChassisMetersMoved());
    SmartDashboard.putNumber("average chassis encoders", getEncodersAverage());
  }

  public ChassisStates evaluateState()
  {
    // TODO: Should we do this evaluation based on actual Drivetrain Encoders, rather than indirectly by the joystick inputs?
    double x = m_stick.getX(), y = m_stick.getY();
    if(y > 0.05){
      return ChassisStates.MOVING_FORWARD;
    }
    if (y < -0.05) {
      return ChassisStates.MOVING_BACK;
    }
    if(x > 0.05){
      return ChassisStates.TURNING_R;
    }
    if(x < -0.05){
      return ChassisStates.TURNING_L;
    }

    return ChassisStates.STOPPED;
  }
}
