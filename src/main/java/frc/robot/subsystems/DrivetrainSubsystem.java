// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

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

  private static final ADIS16470_IMU gyro = new ADIS16470_IMU();

  //private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.sol[0], Constants.sol[1]);
  private final Timer time = new Timer();

  private double s, count; //Time (s) and encoder count
  private double angle;


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

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("Shift:", getShift());
    s = getTime();
    count = getEncoderCount();
  }

  // public void shift(){
  //   solenoid.toggle(); //Might have 2 solenoids
  // }

  // public void shift(final Value val){
  //   solenoid.set(val);
  // }

  public void teleDrive(double x, double y){
    m_drive.arcadeDrive(x, y);
  }
  public void linearDrive(double speed){
    m_drive.arcadeDrive(speed, 0);
  }


  public void setEncoderCount(double count){
    leftM.getSensorCollection().setIntegratedSensorPosition(count, 0);
  }
  public double getEncoderCount(){ 
    return leftM.getSelectedSensorPosition();
  }
  

  public void setAngle(double target){
    target = angle;
  }
  public double getAngle(){
    return gyro.getAngle();
  }

  public double getTime(){ return time.get(); }
  public void restartTime(){ 
    time.reset(); 
    time.start();
  }
  //Josh was here
  // public boolean getShift(){
  //   boolean shift = (solenoid.get() == Value.kForward);
  //   return shift;
  // }
}
