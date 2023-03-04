
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotStates.IntakeStates;
import frc.robot.RobotStates.WristStates;
import frc.robot.utils.PIDController;
import frc.robot.utils.Containers.Multi_CANSparkMax;


public class Intake extends SubsystemBase {
  
  // TODO: These wrist angle values are all made up,
  //       need to measure the actual reading for each.
  public final static double ANGLE_WRIST_IN = 10.0;
  public final static double ANGLE_WRIST_SAFE = 20.0; // This is the minimum angle so the wrist can move past the elevator top
  public final static double ANGLE_WRIST_SHOOT = 70.0;
  public final static double ANGLE_WRIST_PLACE = 90.0;
  public final static double ANGLE_WRIST_INTAKE = 100.0;


  /** Creates a new Intake. */
  Multi_CANSparkMax m_wristSparks, m_intakeSparks;
  MotorControllerGroup m_wristMotors, m_intakeMotors;

  Joystick m_stick;
  XboxController m_controller;
  Solenoid m_solenoid;
  PneumaticHub m_hub;
  PIDController m_wristOriginController;
  IntakeStates m_intakeState;
  WristStates m_wristState;
  DutyCycleEncoder m_wristEncoder;

  public double INTAKE_IN_SPEED = 0.7;
  public double INTAKE_OUT_SPEED = 0.2;
  public double WRIST_SPEED = 0.65;

  double wristValue;
  DecimalFormat df1 = new DecimalFormat("0.##");
  final double wristOrigin = 0;
  final double wrist90 = 0;
  final double wristLimit = 0;

  public Intake(PneumaticHub hub) {
    m_wristSparks = new Multi_CANSparkMax(
      new CANSparkMax(Constants.RobotMap.WRIST1, MotorType.kBrushless),
      new CANSparkMax(Constants.RobotMap.WRIST2, MotorType.kBrushless)
    );
    m_wristSparks.setIdleMode(IdleMode.kBrake);

    CANSparkMax invertedIntakeSpark = new CANSparkMax(Constants.RobotMap.INTAKE2, MotorType.kBrushless);
    invertedIntakeSpark.setInverted(true);
    m_intakeSparks = new Multi_CANSparkMax(
      new CANSparkMax(Constants.RobotMap.INTAKE1, MotorType.kBrushless),
      invertedIntakeSpark
    );
    m_intakeSparks.setIdleMode(IdleMode.kBrake);

    m_wristMotors = m_wristSparks.getMotorControllerGroup();

    m_intakeMotors = m_intakeSparks.getMotorControllerGroup();

    m_stick = new Joystick(0);
    m_controller = new XboxController(1);

    m_hub = hub;

    m_solenoid = hub.makeSolenoid(Constants.RobotMap.INTAKE_SQUEEZE);

    m_wristEncoder = new DutyCycleEncoder(9);
    m_wristEncoder.setConnectedFrequencyThreshold(900);
    m_wristEncoder.reset(); // TODO: The point of this is to be fixed positions, do we want to reset it?
  
    m_wristOriginController = new PIDController(0.04, 0.002, 0, 5, .5, .5,0);
  }

  public DutyCycleEncoder getWristEncoder()
  {
    return m_wristEncoder;
  }

  public void moveWrist(double speed)
  {
    double wristAngle = getWristEncoder().get();
    // This function allows us to feed direct input into the elevator, but safely (i.e. respects potentiometer)
    if (wristAngle < ANGLE_WRIST_IN || wristAngle > ANGLE_WRIST_INTAKE) { return; }

    m_intakeMotors.set(speed);
  }

  public void resetEncoder(int val) {
  }

  public void pSqueeze() {
    m_solenoid.set(true);
    m_intakeState = IntakeStates.IN;
  }

  public void pUnsqueeze() {
    m_solenoid.set(false);
    m_intakeState = IntakeStates.OUT;
  }

  public void runIntakeMotors(double speed)
  {
    m_intakeMotors.set(speed);
  }
  public void iIn(){
    runIntakeMotors(INTAKE_IN_SPEED);
  }
  public void iOut(){
    runIntakeMotors(INTAKE_OUT_SPEED);
  }
  public void iStop(){
    runIntakeMotors(0);
  }

  public void resetWrist(){
    m_wristMotors.set(m_wristOriginController.getOutput(wristValue));
  }

  public boolean wristAtOrigin(){
    return m_wristOriginController.isOnTarget();
  }

  public void teleopPeriodic() {
    m_hub.enableCompressorDigital();

    if(!RobotContainer.TWO_DRIVER_MODE){
      if (m_stick.getRawButton(1)) {
        pUnsqueeze();

      } else {
        pSqueeze();
      }

      if (m_stick.getRawButton(2)){
        //iIn();
        iIn();
        if(m_intakeState == IntakeStates.OUT){
          m_intakeState = IntakeStates.OUT_AND_MOTORS_F;
        }else{
          m_intakeState = IntakeStates.MOTORS_RUNNING_F;
        }
      } else if(m_stick.getRawButton(5)){ 
        iOut();
        if(m_intakeState == IntakeStates.OUT){
          m_intakeState = IntakeStates.OUT_AND_MOTORS_R;
        } else{
          m_intakeState = IntakeStates.MOTORS_RUNNING_F;
        }
      } else{
        iStop();
        if(m_intakeState == IntakeStates.OUT){
          m_intakeState = IntakeStates.OUT_AND_MOTORS_S;
        }
      }

      if (m_stick.getPOV() == 0) {
        m_wristMotors.set(WRIST_SPEED);
        m_wristState = WristStates.ROTATING_IN;
      } else if (m_stick.getPOV() == 180) {
        m_wristMotors.set(-WRIST_SPEED);
        m_wristState = WristStates.ROTATING_OUT;
      } else if(m_stick.getPOV() == 90){
        resetWrist();
      }else {
        m_wristMotors.set(0);
        m_wristState = WristStates.MOTORS_STOPPED;
      }
    }else{
      if(m_controller.getRightTriggerAxis() > .05){
        pUnsqueeze();
      }else{
        pSqueeze();
      }

      if(m_controller.getRightBumper()){
        iIn();
        if(m_intakeState == IntakeStates.OUT){
          m_intakeState = IntakeStates.OUT_AND_MOTORS_F;
        }else{
          m_intakeState = IntakeStates.MOTORS_RUNNING_F;
        }
      }else if(m_controller.getLeftBumper()){
        iOut();
        if(m_intakeState == IntakeStates.OUT){
          m_intakeState = IntakeStates.OUT_AND_MOTORS_R;
        } else{
          m_intakeState = IntakeStates.MOTORS_RUNNING_F;
        }
      }else{
        iStop();
        if(m_intakeState == IntakeStates.OUT){
          m_intakeState = IntakeStates.OUT_AND_MOTORS_S;
        }
      }
      if (m_controller.getPOV() == 0) {
        m_wristMotors.set(-WRIST_SPEED);
        m_wristState = WristStates.ROTATING_IN;
      } else if (m_controller.getPOV() == 180) {
        m_wristMotors.set(WRIST_SPEED);
        m_wristState = WristStates.ROTATING_OUT;
      } else if(m_stick.getPOV() == 90){
        resetWrist();
      }else {
        m_wristMotors.set(0);
        m_wristState = WristStates.MOTORS_STOPPED;
      }
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    wristValue = (Double.valueOf(df1.format(m_wristEncoder.getAbsolutePosition())) * 100)-78;
    SmartDashboard.putString("Wrist State", m_wristState+"");
    SmartDashboard.putString("Intake State", m_intakeState+"");
    SmartDashboard.putNumber("WristEncoder", wristValue);
    SmartDashboard.putBoolean("WristAtOrigin", wristAtOrigin());
  }

  /* Wrist encoder vals
   * -15 is straight
   */
}
