// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.mobility;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.roboActions.drive.Drive;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Gyro;
import frc.robot.utils.PIDController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MobilityOnly extends SequentialCommandGroup {
  /** Creates a new MobilityOnly. */
  frc.robot.utils.PIDController driveForwardController, straightHeadingController;
  public MobilityOnly(Chassis chassis, Gyro gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    driveForwardController = new PIDController(.013, .01, 0, 35, 5, 1, Constants.Values.TICKS_PER_METER*2.5);
    straightHeadingController = new PIDController(.05, .007, 0, .15, .2, .2, 0);
    addCommands(new Drive(chassis,gyro,driveForwardController,straightHeadingController,true));
  }
}
