// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import java.util.Arrays;
import java.util.List;

/** Add your docs here. */
public class ElevatorSubsystemConstants 
{
    public static final int LEFT_MOTOR_ID = 49;
    public static final int RIGHT_MOTOR_ID = 48;
    public static final int GRABBER_MOTOR_ID = 47;

    public static final int CORAL_SENSOR_PROXIMITY_THRESHOLD = 200;

    public static final int NEO550_CURRENT_LIMIT = 30;

    public static final double L1_ENCODER_POSITION = 0.0;
    public static final double L2_ENCODER_POSITION = 0.0;
    public static final double L3_ENCODER_POSITION = 0.0;
    public static final double L4_ENCODER_POSITION = 0.0;

    public static final double HP_ENCODER_POSITION = 0.0;
    public static final double DEFAULT_POSITION = 0.0; //So the carriage doesn't slam into the base.

    public static final double HIGH_ALGAE_POSITION = 0.0;
    public static final double LOW_ALGAE_POSITION = 0.0;

    public static final double GRABBER_SPEED = 0.5;
    public static final double L1_GRABBER_SPEED = .15;
    public static final double INTAKE_GRABBER_SPEED = 0.4;

    public static final double HOMED_CURRENT_DRAW = 75.0;

    public static final double MAX_ACCELERATION = 0;
    public static final double MAX_VELOCITY = 0;
    public static final double AT_SETPOINT_TOLERANCE = 1.0;

    public static final double ARBITRARY_FEEDFORWARD = 0.0;

    public static final List<Double> ALGAE_ELEVATOR_POSITIONS = Arrays.asList(
        
    );
}
