// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

public class AlignWithCargo extends CommandBase {
  
  private PIDController pid = new PIDController(0.01, 0, 0);
  private DriveSubsystem m_drive;
  private Vision m_vision;

  private double error;
  private double leftVelocity;
  private double rightVelocity;
  private int missedFrames = 0;
  private double adjustment;
  
  /** Creates a new MoveToTarget. */
  public AlignWithCargo(Vision v, DriveSubsystem d) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = d;
    m_vision = v;
    addRequirements(m_drive, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset();
    missedFrames = 0 ;
 }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_vision.cargoIsVisible()){
        error = m_vision.cargoDistanceFromCenter();
        adjustment = pid.calculate(error);
        adjustment = Math.signum(adjustment)*Math.min( Math.abs(adjustment), Constants.DriveConstants.CorrectionRotationSpeed / 4.0) ;
        leftVelocity = Constants.DriveConstants.CorrectionRotationSpeed - adjustment ;
        rightVelocity = Constants.DriveConstants.CorrectionRotationSpeed + adjustment;

        m_drive.setWheelSpeeds(
            new MecanumDriveWheelSpeeds(
              leftVelocity, rightVelocity, leftVelocity, rightVelocity)
            );
    } else{
      missedFrames++;
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setWheelSpeeds(
      new MecanumDriveWheelSpeeds(
        0, 0, 0, 0)
    );
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (missedFrames > 10){
      return true;
    }
    else{
      return false;
    }
  }
}