// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.misc.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class AlignLimeShot extends CommandBase {
  /** Creates a new AlignLimeShot. */
  private final LimelightSubsystem lime;
  private final DrivetrainSubsystem drive;
  private final ShooterSubsystem shooter;

  private double targetspeedB, targetspeedT;
  private double currentspeedB, currentSpeedT;

  private double speed;

  public AlignLimeShot(LimelightSubsystem lime, ShooterSubsystem shooter, DrivetrainSubsystem drive) {
    this.lime = lime;
    this.drive = drive;
    this.shooter = shooter;

    addRequirements(lime, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetspeedB = lime.getTargetVelocities()[0];
    targetspeedT = lime.getTargetVelocities()[1];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentspeedB = Math.abs(shooter.getCurrentVelocities()[0]);
    currentSpeedT = Math.abs(shooter.getCurrentVelocities()[1]);

    if(lime.hasTarget()){ //will use values from initialize if ball causes limelight to lose target
      targetspeedB = lime.getTargetVelocities()[0];
      targetspeedT = lime.getTargetVelocities()[1];
    }
    
    shooter.motorOn(-targetspeedB, -targetspeedT);

    if(lime.isWithinRange()){
      if(currentspeedB >= targetspeedB - 550 && currentspeedB <= targetspeedB + 550 
      && currentSpeedT >= targetspeedT - 550 && currentSpeedT <= targetspeedT + 550){
        
        if(shooter.getSensorValues()[0] == true){
          shooter.setConveyor(0.7);
        }
        
        shooter.setIndexer(0.5);
        LedSubsystem.setStrobe(0, 255, 0);
      }
    }
    else if(!lime.isWithinRange()){
      if(lime.hasTarget()){
        if(lime.getHorizOffset() > -.50){
          speed = lime.getHorizOffset() / 40;
          drive.turn(speed + .30);
  
        }
        else if(lime.getHorizOffset() < .50){
          speed = lime.getHorizOffset() / 40;
          drive.turn(speed - .30); 
        }
      }
    }  
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
