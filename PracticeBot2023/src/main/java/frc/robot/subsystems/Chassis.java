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

  ElevatorState elevatorState;

  public Chassis(PneumaticHub hub) {
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
    return (rightLeaderEncoder.getPosition() + (-leftLeaderEncoder.getPosition()))/2;
  }

  public void resetEncoders(){
    rightLeaderEncoder.setPosition(0);
    leftLeaderEncoder.setPosition(0);
  }

  public double getChassisMetersMoved(){
    return getEncodersAverage()/Constants.Values.TICKS_PER_METER;
  }

  public void arcadeDrive(double x, double y) {
    drive.arcadeDrive(x, y);
  }

  public void teleopPeriodic(ElevatorState state){
    double x = stick.getX(), y = -stick.getY();

    if(RobotContainer.TWO_DRIVER_MODE){
      if(stick.getTrigger()){//slows down the chassis for lining up
        x /= 1.7;
        y /= 2.5;
        setBrake();
      }else{
        setCoast();
      }
      if(stick.getRawButton(2)){
        solenoid.set(Value.kForward);
        setBrake();
      }else{
        solenoid.set(Value.kReverse);
        setCoast();
      }
    }else{
      if(stick.getRawButton(7)){
        x /= 1.5;
        y /= 2.5;
      }
    }

    //creates dead zone. Maybe it benefits driving experience
    
    arcadeDrive(x, y);
  }

  // The 'periodic' function is called constantly, even when the robot is not enabled.
  // Therefore, the periodic function should not attempt to move any motors or parts.
  // Instead, it should be used to report data to the SmartDashboard, and to update
  // any Global Variables related to State.
  @Override
  public void periodic() {
    chassisState = evaluateState();
    SmartDashboard.putString("chassis State", chassisState+"");
    SmartDashboard.putNumber("left chassis encoder", leftLeaderEncoder.getPosition());
    SmartDashboard.putNumber("right chassis encoder", rightLeaderEncoder.getPosition());
    SmartDashboard.putNumber("Meters moved", getChassisMetersMoved());
    SmartDashboard.putNumber("average chassis encoders", getEncodersAverage());
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