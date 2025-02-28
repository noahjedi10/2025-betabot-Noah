// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public class DriveSubsystemConstants {
    public static final int FLEFT_DRIVE_ID = 1;
    public static final int FLEFT_STEER_ID = 2;
    public static final int FLEFT_CANCODER = 50;

    public static final int FRIGHT_DRIVE_ID = 3;
    public static final int FRIGHT_STEER_ID = 4;
    public static final int FRIGHT_CANCODER = 51;

    public static final int BLEFT_DRIVE_ID = 5;
    public static final int BLEFT_STEER_ID = 6;
    public static final int BLEFT_CANCODER = 52;

    public static final int BRIGHT_DRIVE_ID = 7;
    public static final int BRIGHT_STEER_ID = 8;
    public static final int BRIGHT_CANCODER = 53;

    public static final double FLEFT_OFFSET = -0.964873915580068 + Math.PI;
    public static final double FRIGHT_OFFSET = -1.109068109641319 + Math.PI;
    public static final double BLEFT_OFFSET = -0.559902987578259 + Math.PI;
    public static final double BRIGHT_OFFSET = -2.626175108860218 + Math.PI;

    public static final String CANIVORE_NAME = "CANIVORE";

    public static final double L = .5525; //placeholder value
    public static final double W = .5525;

    private static final Translation2d[] modulePositions = {
        new Translation2d(L / 2, W / 2),
        new Translation2d(L / 2, -W / 2),
        new Translation2d(-L / 2, W / 2),
        new Translation2d(-L / 2, -W / 2)
    };

    public static final SwerveDriveKinematics M_KINEMATICS = new SwerveDriveKinematics(modulePositions);

    public static final int PIGEON_ID = 10;
}
