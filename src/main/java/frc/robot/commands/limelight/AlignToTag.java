// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;

public class AlignToTag extends Command {
  /** Creates a new AlignToTag. */
  private Limelight mLimelight = new Limelight();
  public AlignToTag() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mLimelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double TX = (Math.abs(Limelight.tx) > 5.5) ? (Math.abs(Limelight.tx)/Limelight.tx * 5.5) : Limelight.tx;
    RobotContainer.drivetrain.applyRequest(()-> RobotContainer.drive.withVelocityX(TX));
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
