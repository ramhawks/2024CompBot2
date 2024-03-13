// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.subsystems.lightingSubsystem;

public class HomeFeederCommand extends Command {
  private final feederSubsystem feeder;
  private final lightingSubsystem lights;

  /** Creates a new HomeFeederCommand. */
  public HomeFeederCommand(feederSubsystem feeder, lightingSubsystem lights) {
    this.feeder = feeder;
    this.lights = lights;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
    addRequirements(lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.homeFeeder();
    lights.rgBlink(0, 0, 0, 0);
    lights.setRGB(191, 245, 66);
    //end(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //feeder.homeFeeder();
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
