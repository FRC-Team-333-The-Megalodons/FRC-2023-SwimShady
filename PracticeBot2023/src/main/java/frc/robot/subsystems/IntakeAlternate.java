// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Metrics;

public class IntakeAlternate extends SubsystemBase {

  
  public int DPAD_UP = 0;
  public int DPAD_UP_RIGHT = 45;
  public int DPAD_RIGHT = 90;
  public int DPAD_DOWN_RIGHT = 135;
  public int DPAD_DOWN = 180;
  public int DPAD_DOWN_LEFT = 225;
  public int DPAD_LEFT = 270;
  public int DPAD_UP_LEFT = 315;

  /** Creates a new IntakeAlternate. */
  CANSparkMax intakeMotor, wristMotor;
  XboxController controller = new XboxController(1);
  DutyCycleEncoder wristEncoder;
  DecimalFormat df1 = new DecimalFormat("0.##");
  double wristValue;
  Elevator m_elevator;

  frc.robot.utils.PIDController intakeController, scoringController, substationController;

  public IntakeAlternate() {
    intakeMotor = new CANSparkMax(Constants.RobotMap.PORT_INTAKE2, MotorType.kBrushless);
    intakeMotor.setInverted(true);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    wristMotor = new CANSparkMax(Constants.RobotMap.PORT_WRIST2, MotorType.kBrushless);
    wristMotor.setIdleMode(IdleMode.kBrake);

    wristEncoder = new DutyCycleEncoder(9);
    wristEncoder.setConnectedFrequencyThreshold(900);
    wristEncoder.reset();

    
    intakeController = new frc.robot.utils.PIDController(7.5, 6, 0, .8, 0.01,0.01,0);
    scoringController = new frc.robot.utils.PIDController(7.5, 6, 0, .8, 0.01,0.01,0);
    substationController = new frc.robot.utils.PIDController(7.5, 6, 0, .8, 0.01,0.01,0);
    
  }

  public void setElevator(Elevator e)
  {
    m_elevator = e;
  }

  public double getRealWristPosition(){
    // It turns out that the `getDistance()` function on the potentiometer
    //  is relative to the point it started, which is not what we need.
    // The `getAbsolutePosiion()` function gets the real absolute point
    //  on the potentiometer, BUT doesn't consider "rollovers" (i.e. when
    //  it goes past 1.00, it goes back to 0.01).
    // The only "saving grace" is that We don't actually need the full rotation
    //  of the wrist to work; it only realistically rotates between -0.2 and 0.8.
    // So, that means if the wrist is somewhere in the "no-mans-land" zone (i.e.
    //  [0.8,0.0) ), given the hardware of this robot, we can assume it's actually
    //  just a rollover.
    double value = wristEncoder.getAbsolutePosition();

    if (value > 0.8) {
      return (value - 1.0);
    }
    return value;
  }

  public void intakeIn(){
    intakeMotor.set(Constants.Intake.INTAKE_SPEED);
  }

  public void passiveIntakeIn()
  {
    intakeMotor.set(Constants.Intake.INTAKE_PASSIVE_SPEED);
  }

  public void eject(){
    intakeMotor.set(Constants.Intake.EJECT_SPEED);
  }

  public void stopIntake(){
    intakeMotor.set(0);
  }

  public void wristIn(){
    
    moveWrist(Constants.Wrist.WRIST_UP_SPEED);
  }

  public void wristOut(){
    moveWrist(Constants.Wrist.WRIST_DOWN_SPEED);
  }

  public void stopWrist(){
    wristMotor.set(0);
  }

  public void wristToIntake(){
    moveWrist(intakeController.getOutput(getRealWristPosition()));
  }

  public void wristToSCore(){
    moveWrist(scoringController.getOutput(getRealWristPosition()));
  }

  public void wristToSubStation(){
    moveWrist(substationController.getOutput(getRealWristPosition()));
  }

  // WRIST UPPER LIMIT WITH CONE IN IT : 0.54
  // WRIST LOWER LIMIT WHEN ELEVATOR UP: 0.07

  // WRIST "Threshold" WHERE ELEVATOR LOW POINT MATTERS = lower than 0.18
  // ELEVATOR low point where wrist needs to unfold: 18.2
  // lowest point elevator can go while wrist pointing down: 3.4
  // Wrist lowest point while 0.28

  public boolean isWristAtMaxUp()
  {
    return getRealWristPosition() >= Constants.Wrist.WRIST_POS_UPPER_LIMIT;
  }

  public  boolean isWristAtMaxDown()
  {
    // This considers the elevator state.
    if (m_elevator.getHeight() >= Constants.Elevator.ELEVATOR_POS_LOWEST_POINT_WRIST_CAN_MOVE) {
      return getRealWristPosition() <= Constants.Wrist.WRIST_POS_LOWER_LIMIT_WHILE_ELEVATOR_UP;
    }

    /*
    if (m_elevator.getHeight() >= Constants.Elevator.ELEVATOR_POS_LOWEST_POINT_ELEVATOR_CAN_GO_WHILE_WRIST_DOWN &&
        m_elevator.getHeight() <= Constants.Elevator.ELEVATOR_POS_LOWEST_POINT_WRIST_CAN_MOVE)
    {
      */
      return getRealWristPosition() <= Constants.Wrist.WRIST_POS_LOWER_LIMIT_WHILE_ELEVATOR_DOWN;
    //}

  }


  public void moveWrist(double speed)
  {
    if (speed > 0) {
      // we're trying to move the wrist up.
      if (isWristAtMaxUp()) {
        stopWrist();
        return;
      }

      // Make sure we cap the speed.
      speed = Math.min(speed, Constants.Wrist.WRIST_UP_SPEED);
    } else if (speed < 0) {
      // we're trying to mvoe the wrist down.
      if (isWristAtMaxDown()) {
        stopWrist();
        return;
      }
      // Make sure we cap the speed
      speed = Math.max(speed, Constants.Wrist.WRIST_DOWN_SPEED);
    }
    wristMotor.set(speed);
  }


  public void teleopPeriodic()
  {
    final String metric_key = "Intake::teleopPeriodic";
    Metrics.startTimer(metric_key);
    teleopPeriodic_impl();
    Metrics.stopTimer(metric_key);
  }

  public void teleopPeriodic_impl()
  {
    if(controller.getRightBumper()){  
      intakeIn();
    }else if(controller.getLeftBumper()){
      eject();
    }else {
      // If no one is holding the intake buttons, and gravity
      //  isn't helping us, do a persistent pull-in to help hold it.
      if (getRealWristPosition() < Constants.Wrist.WRIST_POS_GRAVITY_THRESHOLD)
      {
        passiveIntakeIn();
      } else {
        stopIntake();
      }
    }

    if (Math.abs(controller.getLeftY()) > 0.05) {
      double y = -controller.getLeftY();
      moveWrist(y);
    } else {
      if (controller.getPOV() == DPAD_UP) {
        wristIn();
      }else if (controller.getPOV() == DPAD_DOWN) {
        wristOut();
      }else {
        stopWrist();
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double rawWristValue = getRealWristPosition();
    wristValue = (Double.valueOf(df1.format(rawWristValue)));
    SmartDashboard.putNumber("wrist value", wristValue);
    //SmartDashboard.putNumber("wrist raw", rawWristValue);
  }
}
