// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoEncoderDrive extends CommandBase {
  /** Creates a new AutoEncoderDrive. */
  DrivetrainSubsystem drivetrain;
  double count, speed;
  boolean finished;

  /** @param count - Desired encoder count
   *  @param speed - Desired speed */
  public AutoEncoderDrive(DrivetrainSubsystem drivetrain, double count, double speed) {
    this.drivetrain = drivetrain;
    this.count = count;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setEncoderCount(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.linearDrive(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.

  @Override
  public boolean isFinished() {
    return (drivetrain.getEncoderCount() < count); // Returns true if (encoder count >= target) & vice versa
  }
}
