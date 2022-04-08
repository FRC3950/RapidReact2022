// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class LimelightCenterCommand extends CommandBase {
  /** Creates a new LimelightCenterCommand. */
  private final LimelightSubsystem limelight;
  private final DrivetrainSubsystem drivetrain;

  private final double kP = 0.23;
  private double speed;

  boolean isFinished = false;

  public LimelightCenterCommand(LimelightSubsystem limelight, DrivetrainSubsystem drivetrain) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;

    addRequirements(limelight, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Command start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.hasTarget()){
      if(limelight.getHorizOffset() > -.50){
        speed = limelight.getHorizOffset() / 40;
        drivetrain.turn(speed + .30);
        //right

      }
      else if(limelight.getHorizOffset() < .50){
        speed = limelight.getHorizOffset() / 40;
        drivetrain.turn(speed - .30);

         // <-- Might need to change direction or P values 
      }
      // else {
      //   isFinished = true;
      // }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.turn(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (limelight.getHorizOffset() > -.5 && limelight.getHorizOffset() < 0.5);
  }
}
