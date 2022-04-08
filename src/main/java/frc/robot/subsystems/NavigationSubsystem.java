// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;

import com.kauailabs.navx.*;
import com.kauailabs.navx.frc.AHRS;

public class NavigationSubsystem extends SubsystemBase {
    /** Creates a new NavigationSubsystem. */

    AHRS ahrs ;

    public NavigationSubsystem() {
        ahrs = new AHRS(SerialPort.Port.kUSB1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


    public double getPitch() {
        // wpk temporary change because orientation of NavX is not correct
        return ahrs.getRoll() ;
        // return ahrs.getPitch() ;
    }

    public double getRoll() {
        // wpk temporary change because orientation of NavX is not correct
        return ahrs.getPitch() ;
//        return ahrs.getRoll() ;
    }


}
