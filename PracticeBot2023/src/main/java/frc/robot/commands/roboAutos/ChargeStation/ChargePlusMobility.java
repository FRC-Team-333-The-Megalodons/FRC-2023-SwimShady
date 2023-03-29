// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roboAutos.ChargeStation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.roboActions.drive.Turn;
import frc.robot.commands.roboAutos.ChargeStation.stages.DockAndEngage;
import frc.robot.commands.roboAutos.ChargeStation.stages.UnbalanceForMobility;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.IntakeAlternate;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargePlusMobility extends SequentialCommandGroup {
  /** Creates a new ChargePlusMobility. */
  frc.robot.utils.PIDController turnController, turnResetController;
  public ChargePlusMobility(Elevator elevator, IntakeAlternate intake, Chassis chassis, Gyro gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    turnController = new frc.robot.utils.PIDController(.0043, .008, 0, 90, .35, .45, 0);
    turnResetController = new frc.robot.utils.PIDController(.0043, .008, 0, 90, .35, .45, 0);
    addCommands(
      new Turn(chassis, gyro, 180, turnController, true)
      ,new UnbalanceForMobility(chassis, gyro)
      ,new Turn(chassis, gyro, 0, turnResetController, true)
      ,new DockAndEngage(chassis, gyro)
    );
  }
}
