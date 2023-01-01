// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  PhotonCamera camera = new PhotonCamera("eyeball");
  PhotonPipelineResult result;
  List<PhotonTrackedTarget> tags;
  PhotonTrackedTarget bestTag;
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
    if(result.hasTargets()){
      tags = result.getTargets();
      bestTag = result.getBestTarget();
    }
    else{
      tags = null;
      bestTag = null;
    }
  }
  public double getBestTagDistance(){
    if(bestTag != null)
      return PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_HEIGHT, 
                          Constants.TARGET_HEIGHT[bestTag.getFiducialId()], 
                          Constants.CAMERA_PITCH,
                          Units.degreesToRadians(bestTag.getPitch()));
    else
      return -1;
  }

  public double getBestTagYaw(){
    if( bestTag != null)
      return bestTag.getYaw();
    else{
      return 0.0;
    }
  }

 /*  public Pose2d calcRelativePose(){
    double timestamp = Timer.getFPGATimestamp() - result.getLatencyMillis();
    Transform2d relativePose = new Transform2d(bestTag.getBestCameraToTarget().getTranslation().toTranslation2d(), bestTag.getBestCameraToTarget().getRotation().toRotation2d());
    DifferentialDrivePoseEstimator
  }*/
}
