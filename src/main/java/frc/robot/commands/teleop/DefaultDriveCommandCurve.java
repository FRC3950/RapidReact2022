// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommandCurve extends CommandBase {
  /** Creates a new DefaultDriveCommand. */
  DoubleSupplier x, twist;
  DrivetrainSubsystem drivetrain;
  public DefaultDriveCommandCurve(DoubleSupplier x, DoubleSupplier twist, DrivetrainSubsystem drivetrain) {
    this.x = x;
    this.twist = twist;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //drivetrain.curveDrive(x.getAsDouble(), twist.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
