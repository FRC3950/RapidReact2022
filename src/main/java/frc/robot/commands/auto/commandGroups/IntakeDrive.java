// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auto.autoCommands.AutoEncoderDrive;
import frc.robot.commands.auto.autoCommands.AutoIntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeDrive extends ParallelCommandGroup {
  /** Creates a new IntakeDrive. */
  public IntakeDrive(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IntakeSubsystem intake) {
    addCommands(
      new AutoEncoderDrive(40000, 0.60, drivetrain),
      new AutoIntakeCommand(intake, shooter)
    );
  }
}
