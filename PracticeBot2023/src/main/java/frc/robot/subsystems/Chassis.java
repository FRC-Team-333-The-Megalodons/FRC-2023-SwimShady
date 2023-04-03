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
import frc.robot.utils.Metrics;
import frc.robot.utils.Containers.Multi_CANSparkMax;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */
  Multi_CANSparkMax leftMotors, rightMotors;
  RelativeEncoder rightLeaderEncoder, leftLeaderEncoder;
  MotorControllerGroup rightleader, leftLeader;
  Joystick stick;
  DifferentialDrive drive;
  RobotStates.ChassisStates chassisState;
  PneumaticHub hub;
  DoubleSolenoid solenoid;
  Elevator m_elevator;

  ElevatorState elevatorState;

  public Chassis(PneumaticHub hub, Elevator elevator) {
    leftMotors = new Multi_CANSparkMax(
        new CANSparkMax(Constants.RobotMap.PORT_DRIVE_TRAIN_L_LEADER, MotorType.kBrushless),
        new CANSparkMax(Constants.RobotMap.PORT_DRIVE_TRAIN_L_FOLLOWER1, MotorType.kBrushless),
        new CANSparkMax(Constants.RobotMap.PORT_DRIVE_TRAIN_L_FOLLOWER2, MotorType.kBrushless)
    );
    rightMotors = new Multi_CANSparkMax(
        new CANSparkMax(Constants.RobotMap.PORT_DRIVE_TRAIN_R_LEADER, MotorType.kBrushless),
        new CANSparkMax(Constants.RobotMap.PORT_DRIVE_TRAIN_R_FOLLOWER1, MotorType.kBrushless),
        new CANSparkMax(Constants.RobotMap.PORT_DRIVE_TRAIN_R_FOLLOWER2, MotorType.kBrushless)
    );

    leftLeaderEncoder = leftMotors.getLeader().getEncoder();
    rightLeaderEncoder = rightMotors.getLeader().getEncoder();

    leftLeader = leftMotors.getMotorControllerGroup();
    rightleader = rightMotors.getMotorControllerGroup();

    stick = new Joystick(0);

    drive = new DifferentialDrive(leftLeader, rightleader);

    m_elevator = elevator;

    this.hub = hub;
    solenoid = hub.makeDoubleSolenoid(0, 1);
  }

  public void setBrake(){
    leftMotors.setIdleMode(IdleMode.kBrake);
    rightMotors.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast(){
    leftMotors.setIdleMode(IdleMode.kCoast);
    rightMotors.setIdleMode(IdleMode.kCoast);
  }

  public double getEncodersAverage(){
    return -(rightLeaderEncoder.getPosition() + (-leftLeaderEncoder.getPosition()))/2;
  }

  public void resetEncoders(){
    rightLeaderEncoder.setPosition(0);
    leftLeaderEncoder.setPosition(0);
  }

  public double getChassisMetersMoved(){
    return (getEncodersAverage()/Constants.Values.TICKS_PER_METER);
  }

  public void arcadeDrive(double x, double y) {
    drive.arcadeDrive(x, -y);
  }

  public void lowGear() 
  {
    solenoid.set(Value.kReverse);
  }

  public void highGear()
  {
    solenoid.set(Value.kForward);
  }

  public void teleopPeriodic(){
    final String metric_key = "Chassis::teleopPeriodic";
    Metrics.startTimer(metric_key);
    teleopPeriodic_impl();
    Metrics.stopTimer(metric_key);
  }

  public void teleopPeriodic_impl()
  {
    double x = stick.getX(), y = -stick.getY();

    if(!RobotContainer.TWO_DRIVER_MODE){
      teleopOneDriverMode(x, y);
      return;
    }

    boolean shouldSlowDown = false;

    if(stick.getTrigger()){//slows down the chassis for lining up
      shouldSlowDown = true;
      highGear();
      setBrake();
    }else if(stick.getRawButton(2)){
      lowGear();
      setBrake();
    }else{
      highGear();
      setCoast();
    }

    // If the elevator is high enough up, it's not safe to drive at full speed.
    if (m_elevator.getRightPosition() >= Constants.Elevator.ELEVATOR_BACKUP_UNSAFE) {
      shouldSlowDown = true;
    }

    if (shouldSlowDown) {
      x /= Constants.Chassis.TELEOP_X_SLOWDOWN_DIVISOR;
      y /= Constants.Chassis.TELEOP_Y_SLOWDOWN_DIVISOR;
    }

    arcadeDrive(x, y);
  }

  public void teleopOneDriverMode(double x, double y)
  {
    
    if(stick.getRawButton(7)){
      x /= 1.3;
      y /= 3.1;
    }
  
    arcadeDrive(x, y);
  }

  // The 'periodic' function is called constantly, even when the robot is not enabled.
  // Therefore, the periodic function should not attempt to move any motors or parts.
  // Instead, it should be used to report data to the SmartDashboard, and to update
  // any Global Variables related to State.
  @Override
  public void periodic() {
    final String metric_key = "Chassis::periodic";
    Metrics.startTimer(metric_key);
    periodic_impl();
    Metrics.stopTimer(metric_key);
    SmartDashboard.putNumber("Pressure", hub.getPressure(0));
  }

  public void periodic_impl()
  {
    chassisState = evaluateState();
    //SmartDashboard.putString("chassis State", chassisState+"");
    //SmartDashboard.putNumber("left chassis encoder", leftLeaderEncoder.getPosition());
    //SmartDashboard.putNumber("right chassis encoder", rightLeaderEncoder.getPosition());
    SmartDashboard.putNumber("Meters moved", getChassisMetersMoved());
    SmartDashboard.putNumber("Chassis Encoders", getEncodersAverage());
  }

  public ChassisStates evaluateState()
  {
    // TODO: Should we do this evaluation based on actual Drivetrain Encoders, rather than indirectly by the joystick inputs?
    double x = stick.getX(), y = stick.getY();
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