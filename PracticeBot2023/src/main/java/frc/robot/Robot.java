// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.Metrics;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // To add a new option to the Auto Mode Picker, you need to make a change in
  //  three places: Just search (Ctrl+Shift+F) for AUTO_MODE_PICKER, and you'll
  //  find all three!
  // In this place, just create a new ID name for your auto mode.
  public static final String kNoAuto = "AUTO_NONE";
  public static final String kBalance = "AUTO_BALANCE";
  public static final String kScoreConePlusBalance = "AUTO_HIGH_CONE_PLUS_BALANCE";
  public static final String kScoreCubeHighPlusBalance = "AUTO_HIGH_CUBE_PLUS_BALANCE";
  public static final String kMobilityAuto = "AUTO_MOBILITY_ONLY";
  public static final String kScoreHighCone = "AUTO_SCORE_HIGH_CONE";
  public static final String kScoreHighCube = "AUTO_SCORE_HIGH_CUBE";
  public static final String kConeHighPlusMobility = "AUTO_CONE_HIGH_PLUS_MOBILITY";
  public static final String kCubeHighPlusMobility = "AUTO_CUBE_HIGH_PLUS_MOBILITY";
  public static final String kConeHighPlusPickup = "AUTO_CONE_HIGH_PLUS_PICKUP";
  public static final String kCubeHighPlusPickup = "AUTO_CUBE_HIGH_PLUS_PICKUP";
  public static final String kScoreHighTwiceAuto = "AUTO_SCORE_HIGH_TWICE";
  public static final String kScoreHybridTwiceAuto = "AUTO_SCORE_HYBRID_TWICE";
  public static final String kScoreMidConeOnly = "SCORE_MID_CONE_ONLY";
  public static final String kScoreMidCUbeOnly = "SCORE_MID_CUBE_ONLY";
  public static final String kScoreMidConeMobility = "SCORE_MID_CONE_MOBILITY";
  public static final String kScoreMidCUbeMobility = "SCORE_MID_CUBE_MOBILITY";
  public static final String kScoreMidConeEngage = "SCORE_MID_CONE_ENGAGE";
  public static final String kScoreMidCUbeEngage = "SCORE_MID_CUBE_ENGAGE";
  public static final String kScoreMidConePickUp = "SCORE_MID_CONE_PICKUP";

  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

  private Command m_autonomousCommand;
  Thread m_visionThread;

  private RobotContainer m_robotContainer;

  

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // To add a new option to the Auto Mode Picker, you need to make a change in
    //  three places: Just search (Ctrl+Shift+F) for AUTO_MODE_PICKER, and you'll
    //  find all three!
    // In this place, use addOption to add your Mode ID Name, along with a human-readable name.
    m_autoChooser.setDefaultOption("No Auto", kNoAuto);
    m_autoChooser.addOption("Balance", kBalance);
    m_autoChooser.addOption("ConeHigh + Balance", kScoreConePlusBalance);
    m_autoChooser.addOption("Cube High Balance", kScoreCubeHighPlusBalance);
    m_autoChooser.addOption("Mobility-Only", kMobilityAuto);
    m_autoChooser.addOption("High Cone Only", kScoreHighCone);
    m_autoChooser.addOption("High Cube Only", kScoreHighCube);
    m_autoChooser.addOption("Cone High + Mobility", kConeHighPlusMobility);
    m_autoChooser.addOption("Cube High + Mobility", kCubeHighPlusMobility);
    m_autoChooser.addOption("Cone High + Pickup", kConeHighPlusPickup);
    m_autoChooser.addOption("Cube High PickUp", kCubeHighPlusPickup);
    m_autoChooser.addOption("Score High Twice", kScoreHighTwiceAuto);
    m_autoChooser.addOption("Score Hybrid Twice", kScoreHybridTwiceAuto);
    m_autoChooser.addOption("Score mid cone only", kScoreMidConeOnly);
    m_autoChooser.addOption("Score mid cube only", kScoreMidCUbeOnly);
    m_autoChooser.addOption("Score mid cone mobility", kScoreMidConeMobility);
    m_autoChooser.addOption("Score mid cube mobility", kScoreMidCUbeMobility);
    m_autoChooser.addOption("Score mid cone engage", kScoreMidConeEngage);
    m_autoChooser.addOption("Score mid cube engage", kScoreMidCUbeEngage);
    m_autoChooser.addOption("Score mid cone pickup", kScoreMidConePickUp);
    SmartDashboard.putData("Auto Modes:", m_autoChooser);
 
    m_visionThread = new Thread(
      () -> {
        // Get the UsbCamera from CameraServer
        UsbCamera camera = CameraServer.startAutomaticCapture();
        // Set the resolution
        camera.setResolution(100, 75);

        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink = CameraServer.getVideo();
        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

        // Mats are very memory expensive. Lets reuse this Mat.
        Mat mat = new Mat();

        // This cannot be 'true'. The program will never exit if it is. This
        // lets the robot stop this thread when restarting robot code or
        // deploying.
        while (!Thread.interrupted()) {
          // Tell the CvSink to grab a frame from the camera and put it
          // in the source mat. If there is an error notify the output.
          if (cvSink.grabFrame(mat) == 0) {
            // Send the output the error.
            outputStream.notifyError(cvSink.getError());
            // skip the rest of the current iteration
            continue;
          }
          // Put a rectangle on the image
          Imgproc.rectangle(
            mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
            // Give the output stream a new image to display
            outputStream.putFrame(mat);
          }
        });
        
    m_visionThread.setDaemon(true);
    m_visionThread.start();
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.resetEncoders();
    m_robotContainer.setLEDMode(false);
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    
   
    m_robotContainer.periodic();
    Metrics.log();
    SmartDashboard.putString("Selected Auto:", m_autoChooser.getSelected());
    SmartDashboard.updateValues();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.setChassisBrake();
    m_robotContainer.setLEDMode(false);
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.setChassisBrake();
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    
    CommandScheduler.getInstance().enable();
    CommandScheduler.getInstance().run();
    String selectedAuto = m_autoChooser.getSelected();
    System.out.println("Auto selected: " + selectedAuto);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand(selectedAuto);
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.resetEncoders();
    m_robotContainer.resetTiltOffset(); // Only ever call this in autonomousInit!!!
    m_robotContainer.setChassisBrake();
    m_robotContainer.setLEDMode(false);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // Cancels all running commands at the start of teleopmode.
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().disable();
    m_robotContainer.resetEncoders();
    m_robotContainer.setChassisCoast();
    m_robotContainer.setLEDMode(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.teleopPeriodic();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
