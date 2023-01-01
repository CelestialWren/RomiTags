// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.PIDTagDistance;
import frc.robot.commands.TurnToTag;
import frc.robot.commands.centerAndDistanceAlign;

public class ShuffleboardManager extends SubsystemBase {
  /** Creates a new Shuffleboard. */
  ShuffleboardTab vision = Shuffleboard.getTab("Vision");
  Vision mVision;
  RomiDrivetrain mDrivetrain;
  NetworkTableEntry yaw;
  NetworkTableEntry distance;
  public ShuffleboardManager(RomiDrivetrain drivetrain, Vision vis) {
    Shuffleboard.selectTab("Vision");
    mVision = vis;
    mDrivetrain=drivetrain;
    yaw = Shuffleboard.getTab("Vision").add("Yaw", -1).getEntry();
    distance = Shuffleboard.getTab("Vision").add("Distance", -1).getEntry();
    Shuffleboard.getTab("Vision").add(new centerAndDistanceAlign(mVision, mDrivetrain, .3));
    Shuffleboard.getTab("Vision").add(new TurnToTag(mVision, mDrivetrain));
    Shuffleboard.getTab("Vision").add(new PIDTagDistance(mVision, mDrivetrain, .3));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    yaw.setDouble(mVision.getBestTagYaw());
    distance.setDouble(Units.metersToInches(mVision.getBestTagDistance()));
  }
}
