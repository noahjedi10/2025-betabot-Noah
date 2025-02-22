package frc.robot.utils;

import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class CavbotsPhotonCamera {
    PhotonCamera camera;
    PhotonPoseEstimator estimator;

    // Transform3d cameraInBotSpace = new Transform3d(new Translation3d(0.3048, 0.08255, 0.08), new Rotation3d(0, Math.toRadians(5), 0.0));
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public CavbotsPhotonCamera(String camName, Transform3d cameraInBotSpace) {
        camera = new PhotonCamera(camName);
        estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraInBotSpace);
    }

    private PhotonPipelineResult getLatestResult() {
        // var results = camera.getAllUnreadResults();
        // if(results.isEmpty()) {
        //     return new PhotonPipelineResult();
        // }
        // return results.get(0);
        return camera.getLatestResult();
    }

    public Optional<EstimatedRobotPose> getCameraEstimatedPose3d() {
        var latest = getLatestResult();
        return estimator.update(latest);
    }

    public PoseTimestampPair fetchPose() {  //returns null if there is no new pose
        EstimatedRobotPose ret = null;
        try {
            ret = getCameraEstimatedPose3d().get();
        } catch (NoSuchElementException e) {}
        if(ret != null && getNumTargets() >= 1) {
            return new PoseTimestampPair(ret.estimatedPose.toPose2d(), ret.timestampSeconds);
        }
        return null;
    }

    private double averageCorners(List<TargetCorner> l) { //gets the middle of a tag so the robot can point at it (assuming the camera is centered)
        double divisor = 0.0;
        double xSum = 0.0;
        for(TargetCorner t: l) {
            xSum += t.x;
            divisor += 1.0;
        }
        if(divisor == 0.0) {
            return -1.0;
        }
        return divisor / xSum;
    }

    public double getTargetTagX(int tagID) {
        var result = getLatestResult();
        if(!result.hasTargets()) {
            System.out.println("NO TARGETS");
            return -1.0;
        }
        
        List<PhotonTrackedTarget> targets = result.getTargets();
        for(PhotonTrackedTarget t: targets) {
            if(t.getFiducialId() == tagID) {
                return averageCorners(t.getDetectedCorners());
            }
        }
        return -1.0;
    }

    public int getNumTargets() {
        var result = getLatestResult();
        if(!result.hasTargets()) {
            return 0;
        }
        return result.getTargets().size();
    }
}
