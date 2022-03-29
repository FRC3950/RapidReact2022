// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. **/

 
  private final DoubleSolenoid solenoid = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, 0,4); 
  private final WPI_TalonSRX intake = new WPI_TalonSRX(Constants.intake);
  // private final DoubleSolenoid solTest = new DoubleSolenoid(1,PneumaticsModuleType.REVPH, 6, 7)

  // public static int ballCount = 0;
// false = blocking 
  public IntakeSubsystem() {
    
  }

  @Override
  public void periodic() {
   // SmartDashboard.putBoolean("Intake", getSensor());
    //SmartDashboard.putNumber("Ball count", getBallCount());
    

  }

  public void intake(final double speed){
    if(/*!intakeSensor.get() &&*/ intake.getSupplyCurrent() < 100.00){ //Need to find real current when jamming
      intake.set(speed);
    }
    else if(intake.getSupplyCurrent() >= 100.00){ 
      outtake(0.5);
    }
  }

  /** @param speed (can be pos. or neg.) */ 
  public void outtake(double speed){
    speed = -Math.abs(speed);
    intake.set(speed);
  }

  public void toggleSolenoid(){
    if(solenoid.get() == Value.kReverse){
      solenoid.set(Value.kForward);
    } else {
      solenoid.set(Value.kReverse);
    }
  }

  public void setSolenoid(final Value value){ 
    solenoid.set(value);
  }
  
  /** @param up - kForward if true and kReverse if false */
  public void setSolenoid(boolean up){
    if(up) solenoid.set(Value.kForward);
    if(up == false) solenoid.set(Value.kReverse);
  }

  //public boolean getSensor(){
   //return intakeSensor.get();
   //}

  // public int getBallCount(){
  //   if(intakeSensor.get() == false  && ballCount == 0){
  //     ballCount++;
  //   }
  //   else if(intakeSensor.get() == false && ballCount == 1){
  //     ballCount++;
  //   }
  //   return ballCount;
  // }
}