// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToTag extends ProfiledPIDCommand {
  /** Creates a new TurnToTag. */
  public TurnToTag(Vision vision, RomiDrivetrain mDrivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            Constants.rotKP,
            Constants.rotKI,
            Constants.rotKD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Constants.MAX_ROT_SPEED, Constants.MAX_ROT_ACCEL)),
        // This should return the measurement
        vision::getBestTagYaw,
        // This should return the goal (can also be a constant)
        () -> 0,
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          mDrivetrain.arcadeDrive(0, -output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(mDrivetrain);
    getController().enableContinuousInput(-180, 180);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
