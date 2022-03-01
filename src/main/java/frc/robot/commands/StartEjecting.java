// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterIndex;

public class StartEjecting extends CommandBase {
  
  private Intake m_intake;
  private ShooterIndex m_shooterindex;
  
  /** Creates a new StartEjecting. */
  public StartEjecting(Intake e, ShooterIndex s) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = e;
    m_shooterindex = s;
    addRequirements(m_intake, m_shooterindex);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.startIntake();
    m_shooterindex.startEjecting();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
