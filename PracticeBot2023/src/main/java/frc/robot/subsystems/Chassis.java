// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */
  CANSparkMax rightmotor1, rightmotor2, rightmotor3;
  CANSparkMax leftmotor1, leftmotor2, leftmotor3;

  RelativeEncoder rightLeaderEncoder, leftLeaderEncoder;

  MotorControllerGroup rightleader;
  MotorControllerGroup leftleader;

  Joystick stick;

  DifferentialDrive drive;

  RobotStates.ChassisStates chassisState;
  PneumaticHub hub;
  DoubleSolenoid solenoid;

  ElevatorState elevatorState;

  public Chassis() {
    rightmotor1 = new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_R_LEADER, MotorType.kBrushless);
    rightmotor2 = new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_R_FOLLOWER1, MotorType.kBrushless);
    rightmotor3 = new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_R_FOLLOWER2, MotorType.kBrushless);

    rightLeaderEncoder = rightmotor1.getEncoder();

    rightleader = new MotorControllerGroup(rightmotor1, rightmotor2, rightmotor3);

    leftmotor1 = new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_L_LEADER, MotorType.kBrushless);
    leftmotor2 = new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_L_FOLLOWER1, MotorType.kBrushless);
    leftmotor3 = new CANSparkMax(Constants.RobotMap.DRIVE_TRAIN_L_FOLLOWER2, MotorType.kBrushless);

    leftLeaderEncoder = leftmotor1.getEncoder();

    leftleader = new MotorControllerGroup(leftmotor1, leftmotor2, leftmotor3);

    stick = new Joystick(0);
    drive = new DifferentialDrive(leftleader, rightleader);
    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
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

  public void arcadeDrive(double x, double y)
  {
    drive.arcadeDrive(x, y);
  }

  public void teleopPeriodic(ElevatorState state)
  {
    double x = stick.getX(), y = -stick.getY();

    if(RobotContainer.TWO_DRIVER_MODE){
      if(stick.getTrigger()){//slows down the chassis for lining up
        x /= 2;
        y /= 2;
      }
      if(stick.getRawButton(2)){
        solenoid.set(Value.kForward);
      }else{
        solenoid.set(Value.kReverse);
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
