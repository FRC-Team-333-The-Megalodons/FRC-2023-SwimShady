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
  PIDController wristStraightController;
  IntakeStates intakeState;
  WristStates wristState;
  DutyCycleEncoder wristEncoder;
  ColorSensor m_colorSensor;
  Elevator m_elevator;

  public double INTAKE_SPEED = .7;
  public double WRIST_UP_SPEED = -0.65;
  public double WRIST_UP_SLOW_SPEED = -0.3;
  public double WRIST_DOWN_SPEED = 0.4;

  public int DPAD_UP = 0;
  public int DPAD_UP_RIGHT = 45;
  public int DPAD_RIGHT = 90;
  public int DPAD_DOWN_RIGHT = 135;
  public int DPAD_DOWN = 180;
  public int DPAD_DOWN_LEFT = 225;
  public int DPAD_LEFT = 270;
  public int DPAD_UP_LEFT = 315;



  double wristValue;
  DecimalFormat df1 = new DecimalFormat("0.##");
  final double wristOrigin = 0;
  
  final double WRIST_MAX = 1.02;
  final double WRIST_MIN = 0.74;
  final double WRIST_MIN_WHEN_ELEVATOR_DOWN = 0.82;
  final double WRIST_STRAIGHT = 0.8;
  final double WRIST_GROUND_INTAKE = 0.74;

  final double WRIST_ENCODER_MULTIPLIER = 20;

  public Intake(PneumaticHub hub, ColorSensor colorSensor) {
    wristMotor1 = new CANSparkMax(Constants.RobotMap.PORT_WRIST1, MotorType.kBrushless);
    wristMotor1.setInverted(false);
    wristMotor2 = new CANSparkMax(Constants.RobotMap.PORT_WRIST2, MotorType.kBrushless);

    wristMotor1.setIdleMode(IdleMode.kBrake);
    wristMotor2.setIdleMode(IdleMode.kBrake);

    wrist = new MotorControllerGroup(wristMotor1, wristMotor2);

    intakemotor1 = new CANSparkMax(Constants.RobotMap.PORT_INTAKE1, MotorType.kBrushless);
    intakemotor2 = new CANSparkMax(Constants.RobotMap.PORT_INTAKE2, MotorType.kBrushless);
    
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
    wristStraightController = new PIDController(0.8, 0.05, 0, 2, 
                                                0.002*WRIST_ENCODER_MULTIPLIER,
                                                0.001*WRIST_ENCODER_MULTIPLIER,
                                                WRIST_STRAIGHT*WRIST_ENCODER_MULTIPLIER);
  }

  public void setElevator(Elevator elevator)
  {
    m_elevator = elevator;
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

  public void setWristStaight(){
    moveWrist(-wristStraightController.getOutput(wristValue*WRIST_ENCODER_MULTIPLIER));
  }

  public boolean wristStraight(){
    return wristStraightController.isOnTarget();
  }

  public void teleopPeriodic() {
    hub.enableCompressorDigital();
    if(!RobotContainer.TWO_DRIVER_MODE){
      oneDriverModeTeleopPeriodic();
      return;
    }

    // Actual game Periodic controls below this point

    // Claw Grip (default to "squeeze", open if right trigger held)
    if(controller.getRightTriggerAxis() > 0.1){
      pSqueeze();
    } else if (controller.getLeftTriggerAxis() > 0.1) {
      pUnsqueeze();
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
    if (controller.getPOV() == DPAD_UP) {
      moveWrist(WRIST_UP_SLOW_SPEED);
      wristState = WristStates.ROTATING_IN;
    } else if (controller.getPOV() == DPAD_DOWN) {
      moveWrist(WRIST_DOWN_SPEED);
      wristState = WristStates.ROTATING_OUT;
    }else if(controller.getPOV() >= DPAD_UP_RIGHT && controller.getPOV() <= DPAD_DOWN_RIGHT){
      setWristStaight();
    }else {
      stop();
      wristState = WristStates.MOTORS_STOPPED;
    }
  }

  public void stop()
  {
    wrist.set(0);
  }

  public void moveWrist(double speed)
  {
    if (speed < 0) { // Less than Zero means "wrist up"
      if (isAtMaxUp()) {
        stop();
        return;
      }
      // Make sure we don't go faster that WRIST_UP_SLOW_SPEED!
      speed = Math.max(speed, WRIST_UP_SLOW_SPEED);
    } else { // Greater than Zero means "wrist down"
      if (isAtMaxDown()) {
        stop();
        return;
      }
      // Make sure we don't go faster than WRIST_DOWN_SPEED!
      speed = Math.min(speed, WRIST_DOWN_SPEED);
    }
    wrist.set(speed);
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
      moveWrist(WRIST_UP_SPEED);
      wristState = WristStates.ROTATING_IN;
    } else if (stick.getPOV() == 180) {
      moveWrist(WRIST_DOWN_SPEED);
      wristState = WristStates.ROTATING_OUT;
    } else if(stick.getPOV() == 90){
      setWristStaight();
    }else {
      stop();
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
  public boolean isAtMaxDown(){
    double limit = WRIST_MIN;
    if (m_elevator.isAtMaxDown()) {
      limit = WRIST_MIN_WHEN_ELEVATOR_DOWN;
    }
    
    return getRealWristPosition() <= limit;
  }
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
    SmartDashboard.putBoolean("Wrist straight?", wristStraight());
  }

  /* Wrist encoder vals
   * -15 is straight
   */
}