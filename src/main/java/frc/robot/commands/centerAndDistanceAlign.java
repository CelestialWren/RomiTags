// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.subsystems.Vision;

public class centerAndDistanceAlign extends CommandBase {
  /** Creates a new centerAndDistanceAlign. */
  RomiDrivetrain mDrivetrain;
  Vision mVision;
  double distanceToAlign;
  ProfiledPIDController rotationalController = new ProfiledPIDController(Constants.rotKP, Constants.rotKI, Constants.rotKD, new Constraints(Constants.MAX_ROT_SPEED, Constants.MAX_ROT_ACCEL));
  ProfiledPIDController distanceController = new ProfiledPIDController(Constants.KP, Constants.KI, Constants.KD, new Constraints(Constants.MAX_SPEED, Constants.MAX_ACCEL) );
  public centerAndDistanceAlign(Vision vision, RomiDrivetrain drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    mDrivetrain = drivetrain;
    mVision = vision;
    distanceToAlign = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationalController.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mVision.getBestTagDistance() != -1)
    mDrivetrain.arcadeDrive(distanceController.calculate(mVision.getBestTagDistance(),distanceToAlign), -rotationalController.calculate(mVision.getBestTagYaw(), 0));
    else{
      mDrivetrain.arcadeDrive(0, 0);
    }



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
