// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.roboAutos.ScoreHighTwice;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final boolean TWO_DRIVER_MODE = true;

  PneumaticHub m_hub = new PneumaticHub(Constants.RobotMap.PCM_ID);
  Chassis m_chassis = new Chassis(m_hub);
  Intake m_intake = new Intake(m_hub);
  Elevator m_elevator = new Elevator(m_intake);
  Gyro m_gyro = new Gyro();
  LimeLight m_lLight = new LimeLight();
  ColorSensor sensor = new ColorSensor();
  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new ScoreHighTwice(m_chassis, m_gyro, m_elevator);
  }

  public void periodic() {
    m_elevator.periodic();
    m_intake.periodic();
    m_gyro.periodic();
    m_lLight.periodic();
    sensor.periodic();
  }

  public void teleopPeriodic() {
    m_chassis.teleopPeriodic(m_elevator.getState());
    m_elevator.teleopPeriodic();
    m_intake.teleopPeriodic();
  }

  public void resetEncoders() {
    m_gyro.reset();
    m_chassis.resetEncoders();
  }

  public void setChassisCoast(){
    m_chassis.setCoast();
  }

  public void setChassisBrake(){
    m_chassis.setBrake();
  }
}
