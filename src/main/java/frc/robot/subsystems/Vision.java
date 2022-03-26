// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  DigitalOutput cameraLight;

  public Vision() {
    cameraLight = new DigitalOutput(Constants.VisionConstants.ID_CameraLight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void turnOnLight() {

  }

  public void turnOffLight() {

  }

  public boolean isVisionTargetDetected() {
    //NetworkTableInstance.getEntry("").;
  }

  public double visionTargetDistanceFromCenter() {
    //NetworkTableInstance.getEntry("").''
  }

  public boolean isCargoVisible() {
    //NetworkTableInstance.getEntry("").;
  }

  public double cargoDistanceFromCenter() {
    //NetworkTableInstance.getEntry("").;
  }
}
