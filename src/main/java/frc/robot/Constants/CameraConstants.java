package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraConstants {
    public static final String LOCALIZATION_CAM_ONE_NAME = "lc1";
    public static final String LOCALIZATION_CAM_TWO_NAME = "lc2";

    public static final Transform3d LOCALIZATION_CAM_ONE_OFFSET = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, Math.toRadians(5), 0.0));
    public static final Transform3d LOCALIZATION_CAM_TWO_OFFSET = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, Math.toRadians(5), 0.0));
}