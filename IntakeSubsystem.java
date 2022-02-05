// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.prototypes;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. **/

  private final DigitalInput intakeSensor = new DigitalInput(0); //Placeholder, also might need to make public in future
  private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6,7); 
  private final WPI_TalonSRX intake = new WPI_TalonSRX(10);

  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake", getSensor());
  }

  public void intake(final double speed){
    if(intakeSensor.get() == false && intake.getSupplyCurrent() < 15.00){ //Will need to find actual current when jamming
      intake.set(speed);
    }
    else if(intake.getSupplyCurrent() >= 15.00){ //Will outtake ball if button bound to it is held down 
      intake.set(-speed);
    }
  }

  public void outtake(double speed){
    if(speed > 0.0) speed *= -1;
    intake.set(speed);
  }

  public void toggleSolenoid(){
    solenoid.toggle();
  }

  public void setSolenoid(final Value value){ 
    solenoid.set(value);
  }

  public boolean getSensor(){
    return intakeSensor.get();
  }
}
