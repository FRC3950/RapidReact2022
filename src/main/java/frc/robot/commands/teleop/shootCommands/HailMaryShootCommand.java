// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.shootCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class HailMaryShootCommand extends CommandBase {
  /** Creates a new ShootCommand. */
  ShooterSubsystem shooter;
  IntakeSubsystem intake;
  
  double targetspeedB, targetspeedT;
  double currentspeedB, currentSpeedT;


  public HailMaryShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
    this.shooter = shooter;
    this.intake = intake;
    
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetspeedB = 15833;
    targetspeedT = 19000;

    currentspeedB = Math.abs(shooter.getCurrentVelocities()[0]);
    currentSpeedT = Math.abs(shooter.getCurrentVelocities()[1]);
    

    shooter.motorOn(-targetspeedB, -targetspeedT);
    // shooter.setConveyor(0.5);
    // shooter.setIndexer(0.5);
  
    if(currentspeedB >= targetspeedB - 550 && currentspeedB <= targetspeedB + 550 
    && currentSpeedT >= targetspeedT - 550 && currentSpeedT <= targetspeedT + 550){
      
      if(shooter.getSensorValues()[0] == true){
        shooter.setConveyor(0.7);
      }
      
      shooter.setIndexer(0.5);
    }  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.motorOn(0, 0);
    shooter.setConveyor(0.0);
    intake.intake(0);
    shooter.setIndexer(0);

    //IntakeCommand.numBalls = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
