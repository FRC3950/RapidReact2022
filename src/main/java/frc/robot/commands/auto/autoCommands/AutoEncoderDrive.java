// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.drive.Odometry;

public class AutoEncoderDrive extends CommandBase {
  /** Creates a new AutoEncoderDrive. */
  DrivetrainSubsystem drivetrain;
  double nativeDistance, speed;
  double currentCount = 0;
  boolean finished = false;

  /** @param nativeDistance - Desired encoder count
   *  @param speed - Desired speed (<1.0) */
  public AutoEncoderDrive(final double nativeDistance, double speed, DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    this.nativeDistance = nativeDistance;
    this.speed = speed;
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
    currentCount = Odometry.getAverageEncoderCount();
    
    drivetrain.linearDrive(speed);

    finished = (currentCount >= nativeDistance);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.resetAngle();
    drivetrain.linearDrive(0);
  }

  // Returns true when the command should end.

  @Override
  public boolean isFinished() {
    return finished;
  }
}
