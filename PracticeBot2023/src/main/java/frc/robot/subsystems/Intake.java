// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotStates.IntakeStates;
import frc.robot.RobotStates.WristStates;
import frc.robot.utils.PIDController;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax wristMotor1, wristMotor2,intakemotor1, intakemotor2;
  MotorControllerGroup wrist, intake;
  Joystick stick;
  XboxController controller;
  DoubleSolenoid solenoid;
  PneumaticHub hub;
  PIDController wristOriginController;
  IntakeStates intakeState;
  WristStates wristState;
  DutyCycleEncoder wristEncoder;
  ColorSensor m_colorSensor;

  public double INTAKE_SPEED = .7;
  public double WRIST_UP_SPEED = -0.65;
  public double WRIST_UP_SLOW_SPEED = -0.3;
  public double WRIST_DOWN_SPEED = 0.4;

  double wristValue;
  DecimalFormat df1 = new DecimalFormat("0.##");
  final double wristOrigin = 0;
  
  final double WRIST_MAX = 1.0;
  final double WRIST_MIN = 0.74;

  public Intake(PneumaticHub hub, ColorSensor colorSensor) {
    wristMotor1 = new CANSparkMax(Constants.RobotMap.WRIST1, MotorType.kBrushless);
    wristMotor1.setInverted(false);
    wristMotor2 = new CANSparkMax(Constants.RobotMap.WRIST2, MotorType.kBrushless);

    wristMotor1.setIdleMode(IdleMode.kBrake);
    wristMotor2.setIdleMode(IdleMode.kBrake);

    wrist = new MotorControllerGroup(wristMotor1, wristMotor2);

    intakemotor1 = new CANSparkMax(Constants.RobotMap.INTAKE1, MotorType.kBrushless);
    intakemotor2 = new CANSparkMax(Constants.RobotMap.INTAKE2, MotorType.kBrushless);
    
    intakemotor1.setIdleMode(IdleMode.kBrake);
    intakemotor2.setIdleMode(IdleMode.kBrake);

    intake = new MotorControllerGroup(intakemotor1, intakemotor2);

    stick = new Joystick(0);
    controller = new XboxController(1);

    this.hub = hub;

    solenoid = hub.makeDoubleSolenoid(2, 3);
    m_colorSensor = colorSensor;

    wristEncoder = new DutyCycleEncoder(9);
    wristEncoder.setConnectedFrequencyThreshold(900);
    wristEncoder.reset();
    wristOriginController = new PIDController(0.04, 0.002, 0, 5, .5, .5,0);
  }

  public void resetEncoder(int val) {
    wristMotor1.getEncoder().setPosition(val);
  }

  public void pSqueeze() {
    solenoid.set(Value.kForward);
    intakeState = IntakeStates.IN;
  }

  public void pUnsqueeze() {
    solenoid.set(Value.kReverse);
    intakeState = IntakeStates.OUT;
  }
  public void iIn(){
    intakemotor1.set(-INTAKE_SPEED);
    intakemotor2.set(INTAKE_SPEED);
  }
  public void iOut(){
    intakemotor1.set(.2);
    intakemotor2.set(-.2);
  }
  public void iStop(){
    intake.set(0);
  }

  public void resetWrist(){
    wrist.set(wristOriginController.getOutput(wristValue));
  }

  public boolean wristAtOrigin(){
    return wristOriginController.isOnTarget();
  }

  public void teleopPeriodic() {
    hub.enableCompressorDigital();
    if(!RobotContainer.TWO_DRIVER_MODE){
      oneDriverModeTeleopPeriodic();
      return;
    }

    // Actual game Periodic controls below this point

    // Claw Grip (default to "squeeze", open if right trigger held)
    if(controller.getRightTriggerAxis() > .05){
      pUnsqueeze();
    }else{
      pSqueeze();
    }

    // Claw Wheel Spinners (default to "stopped"; take in if Right Bumper; spit out if Left Bumper)
    //    (also updates IntakeState, might need more work on that)
    if(controller.getRightBumper()){
      iIn();
      if(intakeState == IntakeStates.OUT){
        intakeState = IntakeStates.OUT_AND_MOTORS_F;
      }else{
        intakeState = IntakeStates.MOTORS_RUNNING_F;
      }
    }else if(controller.getLeftBumper()){
      iOut();
      if(intakeState == IntakeStates.OUT){
        intakeState = IntakeStates.OUT_AND_MOTORS_R;
      } else{
        intakeState = IntakeStates.MOTORS_RUNNING_F;
      }
    }else{
      iStop();
      if(intakeState == IntakeStates.OUT){
        intakeState = IntakeStates.OUT_AND_MOTORS_S;
      }
    }

    // Wrist Angle (default to "stopped"; DPAD Up is wrist up, DPAD down is wrist down)
    if (controller.getPOV() == 0 && !isAtMaxUp()) {
      //double speed = WRIST_UP_SPEED;
      //if (isCloseToMaxUp()) { speed = WRIST_UP_SLOW_SPEED; }
      double speed = WRIST_UP_SLOW_SPEED;
      wrist.set(speed);
      wristState = WristStates.ROTATING_IN;
    } else if (controller.getPOV() == 180 && !isAtMaxDown()) {
      double speed = WRIST_DOWN_SPEED;
      //if (isCloseToMaxDown()) { speed /= 2.0; }
      wrist.set(speed);
      wristState = WristStates.ROTATING_OUT;
    //} else if(stick.getPOV() == 90){
    //  resetWrist();
    }else {
      wrist.set(0);
      wristState = WristStates.MOTORS_STOPPED;
    }
  }

  public void oneDriverModeTeleopPeriodic()
  {
    if (stick.getRawButton(1)) {
      pUnsqueeze();

    } else {
      pSqueeze();
    }

    if (stick.getRawButton(2)){
      //iIn();
      iIn();
      if(intakeState == IntakeStates.OUT){
        intakeState = IntakeStates.OUT_AND_MOTORS_F;
      }else{
        intakeState = IntakeStates.MOTORS_RUNNING_F;
      }
    } else if(stick.getRawButton(5)){ 
      iOut();
      if(intakeState == IntakeStates.OUT){
        intakeState = IntakeStates.OUT_AND_MOTORS_R;
      } else{
        intakeState = IntakeStates.MOTORS_RUNNING_F;
      }
    } else{
      iStop();
      if(intakeState == IntakeStates.OUT){
        intakeState = IntakeStates.OUT_AND_MOTORS_S;
      }
    }

    if (stick.getPOV() == 0) {
      wrist.set(WRIST_UP_SPEED);
      wristState = WristStates.ROTATING_IN;
    } else if (stick.getPOV() == 180) {
      wrist.set(WRIST_DOWN_SPEED);
      wristState = WristStates.ROTATING_OUT;
    } else if(stick.getPOV() == 90){
      resetWrist();
    }else {
      wrist.set(0);
      wristState = WristStates.MOTORS_STOPPED;
    }
  }

  public double getRealWristPosition()
  {
    // It turns out that the `getDistance()` function on the potentiometer
    //  is relative to the point it started, which is not what we need.
    // The `getAbsolutePosiion()` function gets the real absolute point
    //  on the potentiometer, BUT doesn't consider "rollovers" (i.e. when
    //  it goes past 1.00, it goes back to 0.01).
    // The only "saving grace" is that We don't actually need the full rotation
    //  of the wrist to work; it only realistically rotates between 0.7 and 1.05.
    // So, that means if the wrist is somewhere in the "no-mans-land" zone (i.e.
    //  [0,0.4) ), given the hardware of this robot, we can assume it's actually
    //  just a rollover.
    double value = wristEncoder.getAbsolutePosition();
    if (value < 0.4) {
      return 1.0+value;
    }
    return value;
  }

  final double WRIST_CLOSE_THRESHOLD = 0.07;
  public boolean isAtMaxUp(){return getRealWristPosition() >= WRIST_MAX;}
  public boolean isCloseToMaxUp() { return Math.abs(getRealWristPosition()-WRIST_MAX) < WRIST_CLOSE_THRESHOLD; }
  public boolean isAtMaxDown(){return getRealWristPosition() <= WRIST_MIN;}
  public boolean isCloseToMaxDown() { return Math.abs(getRealWristPosition()-WRIST_MIN) < WRIST_CLOSE_THRESHOLD; }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double wristPos = getRealWristPosition();
    wristValue = (Double.valueOf(df1.format(wristPos)));
    SmartDashboard.putBoolean("Wrist MIN", isAtMaxDown());
    SmartDashboard.putBoolean("Wrist MAX", isAtMaxUp());
    SmartDashboard.putString("Wrist State", wristState+"");
    SmartDashboard.putString("Intake State", intakeState+"");
    SmartDashboard.putNumber("WristEncoder", wristValue);
    //SmartDashboard.putNumber("WristDistance", wristEncoder.getDistance());
    //SmartDashboard.putNumber("WristGet", wristEncoder.get());
    SmartDashboard.putBoolean("WristAtOrigin", wristAtOrigin());
  }

  /* Wrist encoder vals
   * -15 is straight
   */
}