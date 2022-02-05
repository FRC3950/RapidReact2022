// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.prototypes;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private final WPI_TalonFX leftM = new WPI_TalonFX(1);
  private final WPI_TalonFX leftS = new WPI_TalonFX(15);

  private final WPI_TalonFX rightM = new WPI_TalonFX(2);
  private final WPI_TalonFX rightS = new WPI_TalonFX(14);

  private final DifferentialDrive m_drive;

  private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6,6);
  private final Timer time = new Timer();

  public DrivetrainSubsystem() {

    leftS.follow(leftM);
    rightS.follow(rightM);

    leftM.setNeutralMode(NeutralMode.Brake);
    rightM.setNeutralMode(NeutralMode.Brake);

    m_drive = new DifferentialDrive(leftM, rightM);

    leftM.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightM.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    //PID stuff might go here
  }

  public void shift(){
    solenoid.toggle(); //Might have 2 solenoids
  }

  public void teleDrive(double x, double y){
    m_drive.arcadeDrive(x, y);
  }

  public double getEncoderCount(){ 
    return leftM.getSelectedSensorPosition();
  }
  
  public double getEncoderCount(final WPI_TalonFX fx){ 
    return fx.getSelectedSensorPosition();
  }

  //Will be used for autonomous 
  public void encoderDrive(final double speed, final int units){ 
    do {
      m_drive.arcadeDrive(speed, 0);
    } while(units > getEncoderCount());
  }

  public void setAngle(final double angle){
    final double speed = (angle >= 180) ? -0.5 : 0.5; //Turns left or right depending on angle (p sure left is negative)
    // do{
    //   m_drive.arcadeDrive(0, speed);
    // } while(); New gyro library is not released yet. Would be imu.getYawAngle() < angle
  }  

  public double getTime(){ return time.get(); }
  public void startTime(){ time.start(); }
  public void resetTime(){ time.reset(); }

  public boolean getShift(){
    boolean shift = (solenoid.get() == Value.kForward) ? true : false;
    return shift;
  }

  @Override
  public void periodic() {}
}
