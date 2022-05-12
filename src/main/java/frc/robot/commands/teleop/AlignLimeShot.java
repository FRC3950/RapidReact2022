// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private boolean isSecondBall;
  private double kpLow = 0.025;
  private double kpHigh = 0.015;
  private double kp;

  public AlignLimeShot(LimelightSubsystem lime, ShooterSubsystem shooter, DrivetrainSubsystem drive) {
    this.lime = lime;
    this.drive = drive;
    this.shooter = shooter;

    addRequirements(lime, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

   

    if(drive.getGear() == Value.kForward){
      kp = kpHigh;
    }else {
      kp = kpLow;
    }

    //SmartDashboard.putNumber("kp", kp);


    targetspeedB = lime.getTargetVelocities()[0];
    targetspeedT = lime.getTargetVelocities()[1];

    if(!shooter.getSensorValues()[0] && !shooter.getSensorValues()[1]){
      isSecondBall = true; 
    }
    else {
      isSecondBall = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double kptest = SmartDashboard.getNumber("kp", kp);
    currentspeedB = Math.abs(shooter.getCurrentVelocities()[0]);
    currentSpeedT = Math.abs(shooter.getCurrentVelocities()[1]);

    if(lime.hasTarget()){ //will use values from initialize if ball causes limelight to lose target
      targetspeedB = lime.getTargetVelocities()[0];
      targetspeedT = lime.getTargetVelocities()[1];
    }
    
    // shooter.motorOn(-targetspeedB, -targetspeedT);

    if(lime.isWithinRange() && lime.hasTarget()){

      shooter.motorOn(-targetspeedB, -targetspeedT);

      if(currentspeedB >= targetspeedB - 450 && currentspeedB <= targetspeedB + 250 
      && currentSpeedT >= targetspeedT - 450 && currentSpeedT <= targetspeedT + 250){
        
        if(shooter.getSensorValues()[0] == true){
          shooter.setConveyor(0.7);
        }

        shooter.setIndexer(0.5);
        LedSubsystem.setStrobe(0, 255, 0);
      }
    }
    else if(!lime.isWithinRange()){
      if(lime.hasTarget()){
        if(lime.getHorizOffset() > -.62){
          speed = lime.getHorizOffset() * kp;
          drive.turn(speed + .30);
        }
        else if(lime.getHorizOffset() < .60){
          speed = lime.getHorizOffset() * kp;
          drive.turn(speed - .30); 
        }
      }
    }  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.motorOn(0, 0);
    shooter.setConveyor(0);
    shooter.setIndexer(0);
    drive.turn(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
