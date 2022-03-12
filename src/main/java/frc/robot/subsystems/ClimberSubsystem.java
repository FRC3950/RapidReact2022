// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  WPI_TalonFX master;
  WPI_TalonFX follower;

  DoubleSolenoid whinch;

  public ClimberSubsystem() {
    master = new WPI_TalonFX(Constants.master);
    follower = new WPI_TalonFX(Constants.follower);

    whinch = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, 2,6);

    follower.follow(master);
  }

  @Override
  public void periodic() {}

  public void setMotors(double speed){
    master.set(speed);
  }

  public void setSolenoid(Value value){
    whinch.set(value);
  }

  public void toggleSolenoid(){
    if(whinch.get() == Value.kForward){
      whinch.set(Value.kReverse);
    } 
    else {
      whinch.set(Value.kForward);
    }
  }

}
