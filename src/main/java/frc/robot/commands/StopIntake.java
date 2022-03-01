// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterIndex;
import frc.robot.subsystems.Intake;

public class StopIntake extends CommandBase {

    private Intake m_intake;
    private ShooterIndex m_shooter;

    /** Creates a new StopIntake. */
    public StopIntake(Intake i, ShooterIndex s) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_intake = i;
        m_shooter = s;
        addRequirements(m_intake);
        addRequirements(m_shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.stopIntake();
        m_shooter.stopReceiving();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
