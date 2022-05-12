// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.auto.autoCommands.AutoEncoderDrive;
import frc.robot.commands.auto.autoCommands.AutoIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeDriveUntil extends ParallelRaceGroup {
  /** Creates a new IntakeDrive. */
  int distance;
  public IntakeDriveUntil(final int distance, DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IntakeSubsystem intake) {
    this.distance = distance;
    addCommands(
      new AutoEncoderDrive(distance, 0.60, drivetrain),
      new AutoIntakeCommand(intake, shooter)
    );
  }
}
