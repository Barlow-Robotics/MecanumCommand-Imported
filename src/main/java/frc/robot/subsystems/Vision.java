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

  public void turnOnVisionLight() {
    //turn the green LEDs on and off.
    //The LEDs will be controlled by a digital output from the RoboRio.
    cameraLight.set(true);
  }

  public void turnOffVisionLight() {
    //turn the green LEDs on and off.
    //The LEDs will be controlled by a digital output from the RoboRio.
    //true = on, false = off
    cameraLight.set(false);

  }

  public boolean visionTargetIsVisible() {
    //The data for this will come from the Jetson Nano via network tables.
    return NetworkTableInstance.getDefault().getEntry("vision/target_detected").getBoolean(false);
  }

  public double visionTargetDistanceFromCenter() {
    //returns the number of pixels from the center of the screen to the center of the vision target. 
    //The data for this will come from the Jetson Nano via network tables.
    return NetworkTableInstance.getDefault().getEntry("vision/target_distance_from_center").getDouble(0.0);
  }

  public boolean cargoIsVisible() {
    //The data for this will come from the Jetson Nano via network tables.
    return NetworkTableInstance.getDefault().getEntry("vision/cargo_detected").getBoolean(false);
  }

  public double cargoDistanceFromCenter() {
    //tell how many pixels the piece of cargo is from the center of the screen.
    //The data for this will come from the Jetson Nano via network tables.
    return NetworkTableInstance.getDefault().getEntry("vision/cargo_distance_from_center").getDouble(0.0);
  }

}