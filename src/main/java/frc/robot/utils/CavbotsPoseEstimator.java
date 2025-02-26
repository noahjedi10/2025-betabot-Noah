package frc.robot.utils;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.DriveSubsystemConstants;
import frc.robot.subsystems.DriveSubsystem;

public class CavbotsPoseEstimator {
    private SwerveDrivePoseEstimator estimator;

    CavbotsPhotonCamera[] localizationCameras = {
        new CavbotsPhotonCamera(CameraConstants.LOCALIZATION_CAM_ONE_NAME, CameraConstants.LOCALIZATION_CAM_ONE_OFFSET),
        new CavbotsPhotonCamera(CameraConstants.LOCALIZATION_CAM_TWO_NAME, CameraConstants.LOCALIZATION_CAM_TWO_OFFSET)
    };

    public CavbotsPoseEstimator(DriveSubsystem driveSubsystem, Pose2d initialPose2d) {
        estimator = new SwerveDrivePoseEstimator(DriveSubsystemConstants.M_KINEMATICS, driveSubsystem.getAngle(), driveSubsystem.getModulePositions(), initialPose2d);
    }

    public Pose2d getPose2d() {
        return estimator.getEstimatedPosition();
    }

    private void tryVisionUpdateWithCamera(CavbotsPhotonCamera c) {
        PoseTimestampPair poseTimestampPair = c.fetchPose();
        if(poseTimestampPair != null) {
            estimator.addVisionMeasurement(poseTimestampPair.pose, poseTimestampPair.latency);
        } else {
            // System.out.println("Failed to update with a camera");
        }
    }

    public void updateWithAllAvailableVisionMeasurements() {
        for(CavbotsPhotonCamera c: localizationCameras) {
            tryVisionUpdateWithCamera(c);
        }
    }

    public void updateWithVisionAndOdometry(Rotation2d gyroAngle, SwerveModulePosition[] position) {
        updateWithAllAvailableVisionMeasurements();
        estimator.update(gyroAngle, position);
    }

    public void resetEstimatorPosition(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
        estimator.resetPosition(gyroAngle, modulePositions, pose);
    }
}