// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

/** @author Pat (Possibly the greatest code I have ever written)*/
public class DashboardSettings extends SubsystemBase {
  /** Creates a new DashboardSettings. */
  public static enum ViewMode{
    INFO, 
    NORMAL
  }

  private final SendableChooser<ViewMode> modeSelect = new SendableChooser<>();
  private final SendableChooser<SubsystemBase> subsystemSelect = new SendableChooser<>();
  
  private static ViewMode selectedMode;
  private static SubsystemBase selectedSubsystem;

  public DashboardSettings(SubsystemBase...subsystems) { 
    for(final SubsystemBase s : subsystems){
      subsystemSelect.addOption(s.toString(), s);
    }
    modeSelect.addOption("Info", ViewMode.INFO);
    modeSelect.addOption("Normal", ViewMode.NORMAL);

    SmartDashboard.putData("Subsystem", subsystemSelect);
    SmartDashboard.putData("Mode", modeSelect);
  }

  public static ViewMode getMode(){
    return selectedMode;
  }

  public static SubsystemBase getSubsystemSelected(){
    return selectedSubsystem;
  }

  /** See {@link DrivetrainSubsystem} for (untested) implementation */
  public static boolean isInInfoMode(final SubsystemBase subsystem){
    if(getMode() != null && getSubsystemSelected() != null){
      return (getMode().equals(ViewMode.INFO) && getSubsystemSelected().equals(subsystem));
    }
    else {
      return false;
    }
  }

  @Override
  public void periodic() {
    selectedMode = modeSelect.getSelected();
    selectedSubsystem = subsystemSelect.getSelected();
  }
}
