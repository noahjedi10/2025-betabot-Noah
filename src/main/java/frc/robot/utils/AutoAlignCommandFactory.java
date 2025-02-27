// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlgaeGrabberSubsystemConstants;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Constants.PathingConstants;
import frc.robot.commands.AlgaeGrabberStates.EjectAlgaeCommand;
import frc.robot.commands.AlgaeGrabberStates.StowAlgaeCommand;
import frc.robot.commands.AlgaeGrabberStates.AutoAlign.AlgaeGrabberAndElevatorPositionAndIntakeCommand;
import frc.robot.commands.AutoAlign.FollowPrecisePathAndRaiseElevatorAndScoreCommand;
import frc.robot.commands.AutoAlign.FollowPrecisePathCommand;
import frc.robot.commands.ElevatorStates.ElevatorGoToPositionAndEndCommand;
import frc.robot.commands.ElevatorStates.ElevatorRunGrabberAndGoToPositionCommand;
import frc.robot.commands.ElevatorStates.AutonomousElevatorCommands.ExtendToHeightThenScoreCommand;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/** Add your docs here. */
public class AutoAlignCommandFactory {
    static List<Pose2d> leftBlueAllianceScoringPositions = new ArrayList<>();
    static List<Pose2d> rightBlueAllianceScoringPositions = new ArrayList<>();

    static List<Pose2d> rightRedAllianceScoringPositions = new ArrayList<>();
    static List<Pose2d> leftRedAllianceScoringPositions = new ArrayList<>();

    static List<Pose2d> blueAllianceAlgaePositions = new ArrayList<>();
    static List<Pose2d> redAllianceAlgaePositions = new ArrayList<>();

    static boolean initialized = false;

    public static List<Pose2d> mirrorBlueSidedPoseList(List<Pose2d> list) {
        List<Pose2d> ret = new ArrayList<>();
        for(Pose2d pose: list) {
            Pose2d poseToAdd = new Pose2d(PathingConstants.FIELD_WIDTH_METERS - pose.getX(), PathingConstants.FIELD_HEIGHT_METERS - pose.getY(), pose.getRotation().plus(Rotation2d.fromDegrees(180)));
            ret.add(poseToAdd);
        }
        return ret;
    }

    @SuppressWarnings("unused")
    private static void display(List<Pose2d> list) {
        for(int i = 0; i < list.size(); i++) {
            Field2d field = new Field2d();
            field.setRobotPose(list.get(i));
            SmartDashboard.putData(String.valueOf(i), field);
        }
    }

    public static List<Pose2d> applyXYOffsetsToPoseList(double x, double y, List<Pose2d> originalPoseList) {
        List<Pose2d> ret = new ArrayList<>();
        for(int i = 0; i < originalPoseList.size(); i++) {
            // Field2d field = new Field2d();
            Pose2d originalPose = originalPoseList.get(i);
            double offset = Math.atan2(x, y);
            double magOffset = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
            double poseOrientationRadians = originalPose.getRotation().getRadians();

            double newX = originalPose.getX() + ((Math.cos(poseOrientationRadians + offset) * magOffset));
            double newY = originalPose.getY() + ((Math.sin(poseOrientationRadians + offset) * magOffset));

            Pose2d np = new Pose2d(newX, newY, originalPose.getRotation());

            // field.setRobotPose(np);
            // SmartDashboard.putData(String.valueOf(i), field);

            ret.add(np);
        }
        return ret;
    }

    public static void initalize() {
        if(!initialized) {
            initialized = true;
            leftBlueAllianceScoringPositions = applyXYOffsetsToPoseList(PathingConstants.X_OFFSET, PathingConstants.Y_OFFSET, PathingConstants.LEFT_BLUE_SIDED_SCORING_POSITIONS);
            rightBlueAllianceScoringPositions = applyXYOffsetsToPoseList(PathingConstants.X_OFFSET, PathingConstants.Y_OFFSET, PathingConstants.RIGHT_BLUE_SIDED_SCORING_POSITIONS);

            leftRedAllianceScoringPositions = mirrorBlueSidedPoseList(leftBlueAllianceScoringPositions);
            rightRedAllianceScoringPositions = mirrorBlueSidedPoseList(rightBlueAllianceScoringPositions);

            blueAllianceAlgaePositions = applyXYOffsetsToPoseList(PathingConstants.ALGAE_X_OFFSET, PathingConstants.ALGAE_Y_OFFSET, PathingConstants.BLUE_SIDED_ALGAE_INTAKE_POSITIONS);
            redAllianceAlgaePositions = mirrorBlueSidedPoseList(blueAllianceAlgaePositions);
            display(blueAllianceAlgaePositions);
        }
    }

    public static Pose2d getClosestPose(Pose2d origin, boolean onRedAlliance, boolean left) {
        initalize();
        List<Pose2d> poseList;

        if(left) { //I'm sorry
            if(onRedAlliance) {
                poseList = leftRedAllianceScoringPositions;
            } else {
                poseList = leftBlueAllianceScoringPositions;
            }
        } else {
            if(onRedAlliance) {
                poseList = rightRedAllianceScoringPositions;
            } else {
                poseList = rightBlueAllianceScoringPositions;
            }
        }

        return origin.nearest(poseList);
    }

    public static Pose2d getClosestAlgaeIntakePose(Pose2d origin, boolean onRedAlliance) {
        List<Pose2d> poseList = (onRedAlliance) ? redAllianceAlgaePositions: blueAllianceAlgaePositions;
        return origin.nearest(poseList);
    }

    public static double getAlgaeElevatorEncoderPosition(Pose2d origin, boolean onRedAlliance) {
        Pose2d nearestPose = getClosestAlgaeIntakePose(origin, onRedAlliance);
        int index = (onRedAlliance) ? redAllianceAlgaePositions.indexOf(nearestPose): blueAllianceAlgaePositions.indexOf(nearestPose);
        boolean isHigh = index % 2 == 0; //This works because I found the poses in a circle and the first pose I found was high. It's stupid.

        return (isHigh) ? ElevatorSubsystemConstants.HIGH_ALGAE_POSITION: ElevatorSubsystemConstants.LOW_ALGAE_POSITION;
    }

    public static Command getAutoAlignDriveCommand(DriveSubsystem driveSubsystem, Pose2d currentPosition, boolean onRedAlliance, boolean onLeftSide) {
        initalize();
        Pose2d goalPose = getClosestPose(currentPosition, onRedAlliance, onLeftSide);

        return new FollowPrecisePathCommand(driveSubsystem, goalPose);
    }

    public static Command getAutoAlignDriveCommandAlgae(DriveSubsystem driveSubsystem, Pose2d currentPosition, Pose2d goalPose, boolean onRedAlliance) {
        return new FollowPrecisePathCommand(driveSubsystem, goalPose);
    }

    public static Command getAutoAlignAndScoreCommand(Pose2d currentPosition, ElevatorSubsystem elevatorSubsystem, DriveSubsystem driveSubsystem, double elevatorEncoderPosition, boolean onRedAlliance, boolean onLeftSide) {
        System.out.println(onLeftSide);
        return new SequentialCommandGroup(
            getAutoAlignDriveCommand(driveSubsystem, currentPosition, onRedAlliance, onLeftSide),
            new ExtendToHeightThenScoreCommand(elevatorSubsystem, elevatorEncoderPosition)
        );
    }

    public static Command getAutoAlignAndScoreCommand(Pose2d currentPosition, ElevatorSubsystem elevatorSubsystem, DriveSubsystem driveSubsystem, double elevatorEncoderPosition, boolean onRedAlliance, boolean onLeftSide, double grabberSpeed) {
        System.out.println(onLeftSide);
        return new SequentialCommandGroup(
            getAutoAlignDriveCommand(driveSubsystem, currentPosition, onRedAlliance, onLeftSide),
            new ExtendToHeightThenScoreCommand(elevatorSubsystem, elevatorEncoderPosition, grabberSpeed)
        );
    }

    public static Command getAutoAlignAndScoreCommandParallel(Pose2d currentPosition, ElevatorSubsystem elevatorSubsystem, DriveSubsystem driveSubsystem, double elevatorEncoderPosition, boolean onRedAlliance, boolean onLeftSide) {
        System.out.println(onLeftSide);
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                getAutoAlignDriveCommand(driveSubsystem, currentPosition, onRedAlliance, onLeftSide),
                new ElevatorGoToPositionAndEndCommand(elevatorSubsystem, elevatorEncoderPosition)
            ),
            new ElevatorRunGrabberAndGoToPositionCommand(elevatorSubsystem, elevatorEncoderPosition)
        );
    }

    public static Command getAutoAlignAndScoreCommandParallel(Pose2d currentPosition, ElevatorSubsystem elevatorSubsystem, DriveSubsystem driveSubsystem, double elevatorEncoderPosition, boolean onRedAlliance, boolean onLeftSide, double grabberSpeed) {
        System.out.println(onLeftSide);
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                getAutoAlignDriveCommand(driveSubsystem, currentPosition, onRedAlliance, onLeftSide),
                new ElevatorGoToPositionAndEndCommand(elevatorSubsystem, elevatorEncoderPosition)
            ),
            new ElevatorRunGrabberAndGoToPositionCommand(elevatorSubsystem, elevatorEncoderPosition, grabberSpeed)
        );
    }

    public static Command getAutoAlignAndScoreCommandParallelV2(Pose2d currentPosition, ElevatorSubsystem elevatorSubsystem, DriveSubsystem driveSubsystem, double elevatorEncoderPosition, boolean onRedAlliance, boolean onLeftSide) {
        initalize();
        Pose2d goalPose = getClosestPose(currentPosition, onRedAlliance, onLeftSide);

        return new FollowPrecisePathAndRaiseElevatorAndScoreCommand(driveSubsystem, elevatorSubsystem, elevatorEncoderPosition, goalPose);
    }

    public static Command getAutoAlignAndScoreCommandParallelV2(Pose2d currentPosition, ElevatorSubsystem elevatorSubsystem, DriveSubsystem driveSubsystem, double elevatorEncoderPosition, boolean onRedAlliance, boolean onLeftSide, double grabberSpeed) {
        initalize();
        Pose2d goalPose = getClosestPose(currentPosition, onRedAlliance, onLeftSide);

        return new FollowPrecisePathAndRaiseElevatorAndScoreCommand(driveSubsystem, elevatorSubsystem, elevatorEncoderPosition, goalPose, grabberSpeed);
    }

    public static Command getAutoAlignAndAlgaeIntakeParallel(Pose2d currentPosition, ElevatorSubsystem elevatorSubsystem, AlgaeGrabberSubsystem algaeGrabberSubsystem, DriveSubsystem driveSubsystem, boolean onRedAlliance, boolean ejectAfterIntaking) {
        initalize();
        Pose2d goalPose = getClosestAlgaeIntakePose(currentPosition, onRedAlliance);
        double elevatorEncoderPosition = getAlgaeElevatorEncoderPosition(goalPose, onRedAlliance);

        Command driveAndIntake = new ParallelCommandGroup(
            getAutoAlignDriveCommandAlgae(driveSubsystem, currentPosition, goalPose, onRedAlliance),
            new AlgaeGrabberAndElevatorPositionAndIntakeCommand(elevatorSubsystem, algaeGrabberSubsystem, elevatorEncoderPosition, AlgaeGrabberSubsystemConstants.ALGAE_REMOVAL_ENCODER_POSITION) 
        );

        if(ejectAfterIntaking) {
            return driveAndIntake.andThen(new EjectAlgaeCommand(algaeGrabberSubsystem, elevatorSubsystem));
        }

        return driveAndIntake.andThen(new StowAlgaeCommand(algaeGrabberSubsystem, elevatorSubsystem));
    }
}
