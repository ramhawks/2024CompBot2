// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.feederSubsystem;

public class simpleAutoShootAndDrive extends Command {
  private final SwerveSubsystem drivetrain;
  private final feederSubsystem feeder;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  public double startTime;
  

  /** Creates a new simpleautoshootandrive. */
  public simpleAutoShootAndDrive(SwerveSubsystem drivetrain, feederSubsystem feeder) {
    this.drivetrain = drivetrain;
    this.feeder = feeder;
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        
    // 1. Get joystic inputs
    double xSpeed = 1;
    double ySpeed = 0;
    double turningSpeed = 0;

    // 2. Apply deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    // 3. Make the driving smoother
    if (RobotContainer.driverController.getRightBumper()){
      xSpeed = xLimiter.calculate(xSpeed) * (DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.kSlowButtonDriveModifier);
      ySpeed = yLimiter.calculate(ySpeed) * (DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.kSlowButtonDriveModifier);
      turningSpeed = turningLimiter.calculate(turningSpeed) * (DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * DriveConstants.kSlowButtonTurnModifier);
    }else{
      xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
      ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
      turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    }



    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    /*if (fieldOritentedFunction.get()) {
      // Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    } else { */
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
  //}

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to the wheels
    drivetrain.setModuleStates(moduleStates);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - startTime > 5)
        return true;
    return false;
  }
  
}
