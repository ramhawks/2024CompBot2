// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.subsystems.lightingSubsystem;



public class DepositAmpCommand extends Command {
  private final feederSubsystem feeder;
  private final lightingSubsystem lighting;

  /** Creates a new DepositAmpCommand. */
  public DepositAmpCommand(feederSubsystem feeder, lightingSubsystem lighting) {
    this.feeder = feeder;
    this.lighting = lighting;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
    addRequirements(lighting);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.depositAmp();
    lighting.setRGB(66,173,245);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
