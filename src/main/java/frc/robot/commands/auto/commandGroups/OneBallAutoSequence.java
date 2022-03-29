// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.autoCommands.AutoEncoderDrive;
import frc.robot.commands.auto.autoCommands.AutoShootCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallAutoSequence extends SequentialCommandGroup {
  /** Creates a new OneBallAutoSequence. */
  public OneBallAutoSequence( DrivetrainSubsystem drivetrain, ShooterSubsystem shooter) {
    addCommands(
      new AutoShootCommand(11027, 9990, 4.0, shooter),
      new AutoEncoderDrive(45000, 0.6, drivetrain)
    );
  }
}
