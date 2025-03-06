// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDSubsystemConstants;

public class LEDSubsystem extends SubsystemBase {
  AddressableLED addressableLED = new AddressableLED(LEDSubsystemConstants.LED_PORT);
  AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDSubsystemConstants.BUFFER_LENGTH);

  AddressableLEDBufferView viewRight = buffer.createView(0, 9);
  AddressableLEDBufferView viewLeft = buffer.createView(14, LEDSubsystemConstants.BUFFER_LENGTH - 1);
  AddressableLEDBufferView viewMiddle = buffer.createView(15, 17);

  LEDPattern onPattern = LEDPattern.solid(Color.kGreen).blink(Seconds.of(LEDSubsystemConstants.BLINK_ON_TIME));
  LEDPattern offPattern = LEDPattern.solid(Color.kRed);


  public LEDSubsystem() {
    addressableLED.setLength(buffer.getLength());
    addressableLED.setData(buffer);
    addressableLED.start();
  }

  public void leftOn() {
    setLeft(onPattern);
    setRight(offPattern);
    updateBuffer();
  }

  public void rightOn() {
    setRight(onPattern);
    setLeft(offPattern);
    updateBuffer();
  }

  public void setMiddle(LEDPattern pattern) {
    pattern.applyTo(viewMiddle);
  }

  public void setLeft(LEDPattern pattern) {
    pattern.applyTo(viewLeft);
  }

  public void setRight(LEDPattern pattern) {
    pattern.applyTo(viewRight);
  }

  public void updateBuffer() {
    addressableLED.setData(buffer);
  }

  public void stopAll() {
    setMiddle(LEDPattern.kOff);
    setRight(LEDPattern.kOff);
    setLeft(LEDPattern.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
