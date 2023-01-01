// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDTagDistance extends ProfiledPIDCommand {
  /** Creates a new distanceFromTag. */
  public PIDTagDistance(Vision vision, RomiDrivetrain drivetrain, double distanceMeters) {
    super(
        // The controller that the command will use
        new ProfiledPIDController(Constants.KP, Constants.KI, Constants.KD, new Constraints(Constants.MAX_SPEED, Constants.MAX_ACCEL)),
        // This should return the measurement
        vision::getBestTagDistance,
        // This should return the setpoint (can also be a constant)
        () -> distanceMeters,
        // This uses the output
        (output,setpoint) -> {
          // Use the output here
          if(vision.getBestTagDistance() != -1)
          drivetrain.arcadeDrive(-output, 0);
          else 
            drivetrain.arcadeDrive(0, 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
