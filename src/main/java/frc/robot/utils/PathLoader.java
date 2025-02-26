// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathingConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class PathLoader {
    static SendableChooser<String> chooser = new SendableChooser<String>();
    private static Supplier<Pose2d> poseSupplier;
    
    static String[] validAutonPaths = {
  
    };

    public static PathPlannerPath getPath(String path) {
        try {
            return PathPlannerPath.fromPathFile(path);
        }  catch(Exception e) {
            return null;
        }
    }

    public static Boolean getShouldFlipPath() {
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
            return alliance.get() == Alliance.Red;
        }
        return false;
    }

    public static void configureAutoBuilder(DriveSubsystem driveSub, CavbotsPoseEstimator estimator) {
        Consumer<Pose2d> resetPose = pose -> {
            estimator.resetEstimatorPosition(driveSub.getAngle(), driveSub.getModulePositions(), pose);
        };

        Consumer<ChassisSpeeds> drivelol = speeds -> driveSub.autoDrive(speeds);

        PPHolonomicDriveController holonomicDriveController = new PPHolonomicDriveController(
            new PIDConstants(5.0),
            new PIDConstants(5.0)
        );

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch(Exception e) {
            config = null;
            System.err.println("failed to load path");
        }

        poseSupplier = estimator::getPose2d;

        AutoBuilder.configure(
                poseSupplier, 
                resetPose,
                driveSub::getRobotRelativeChassisSpeeds,
                drivelol,
                holonomicDriveController,
                config,
                () -> getShouldFlipPath(),
                driveSub);
    }

    public static void configureDynamicObstacles(Pose2d[] botObstacleCenters, Translation2d currentBotPos) {}

    public static Command loadAuto(String name) {
        return new PathPlannerAuto(name);
    }

    public static Command pathfindToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, PathingConstants.PATHFINDING_CONSTRAINTS);
    }

    public static Command generateDirectPath(Pose2d goal) { //This Probably Works, should test
        Pose2d origin = poseSupplier.get();
        Rotation2d angleOfTravel = Rotation2d.fromDegrees(Math.atan2(goal.getY() - origin.getY(), goal.getX() - origin.getX()));

        List<Waypoint> list = PathPlannerPath.waypointsFromPoses(
            new Pose2d(origin.getTranslation(), angleOfTravel),
            new Pose2d(goal.getTranslation(), angleOfTravel)
        );

        PathPlannerPath path = new PathPlannerPath(list, PathingConstants.PATHFINDING_CONSTRAINTS, null, new GoalEndState(0.0, goal.getRotation()));
        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

    public static void initSendableChooser() {
        chooser.setDefaultOption(validAutonPaths[0], validAutonPaths[0]);
        for(String v: validAutonPaths) {
            if(v == validAutonPaths[0]) {
                continue;
            }
            chooser.addOption(v, v);
        }
        SmartDashboard.putData("Autonomous type", chooser);
    }

    private static String getAutoName() {
        return chooser.getSelected();
    }

    public static Command getChosenAuton() {
        String selected = getAutoName();
        SmartDashboard.putString("Selected auto", selected);
        return loadAuto(selected);
    }
}