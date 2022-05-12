// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.drive.Odometry;


public class AutoTurn extends CommandBase {
  DrivetrainSubsystem drivetrain;
  double angle, error;
  double kp = 0.0027;
  boolean isFinished = false;

  /** Creates a new AutoTurn. Angle should be entered negative! */
  public AutoTurn(double angle, DrivetrainSubsystem drivetrain) {
    this.angle = angle;
    this.drivetrain = drivetrain;


    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.zeroHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    error = kp*(angle - drivetrain.getAngle());

    if(drivetrain.getAngle() > angle + .5){
      drivetrain.turn(-error + 0.3);
    }
    else if(drivetrain.getAngle() < angle -.5){
      drivetrain.turn(-error -0.3);
    }
    else{
      drivetrain.turn(0);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Auto turn command finished");
    drivetrain.resetHeadingEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
