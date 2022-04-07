// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LimeShot extends CommandBase {
  private final LimelightSubsystem lime;
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake; 

  double currentspeedB, currentSpeedT;
  double targetspeedB, targetspeedT;

  public LimeShot(LimelightSubsystem lime, ShooterSubsystem shooter, IntakeSubsystem intake) {
    this.lime = lime;
    this.shooter = shooter;
    this.intake = intake;

    addRequirements(lime, shooter, intake);
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
      }
    }  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.motorOn(0, 0);
    shooter.setConveyor(0.0);
    shooter.setIndexer(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
