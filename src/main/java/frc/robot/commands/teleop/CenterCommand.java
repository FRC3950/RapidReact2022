// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class CenterCommand extends CommandBase {
  /** Creates a new CenterCommand. */
  private final DrivetrainSubsystem drivetrain;
  private final LimelightSubsystem limelight;
  private boolean withinRange = false;
  
  public CenterCommand(DrivetrainSubsystem drivetrain, LimelightSubsystem limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.turn(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return withinRange;
  }
}
