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

  AddressableLEDBufferView viewLeft = buffer.createView(0, 8);
  AddressableLEDBufferView viewRight = buffer.createView(16, LEDSubsystemConstants.BUFFER_LENGTH - 1);
  AddressableLEDBufferView viewMiddle = buffer.createView(9, 15);

  public LEDSubsystem() {
    addressableLED.setLength(buffer.getLength());
    addressableLED.setData(buffer);
    addressableLED.start();
  }

  public void setLeftColor(Color color, boolean blink) {
    LEDPattern pattern = LEDPattern.solid(color);
    if(blink)
      pattern.blink(Seconds.of(LEDSubsystemConstants.BLINK_ON_TIME));
    pattern.applyTo(viewLeft);
    addressableLED.setData(buffer);
  }

  public void setRightColor(Color color, boolean blink) {
    LEDPattern pattern = LEDPattern.solid(color);
    if(blink)
      pattern.blink(Seconds.of(LEDSubsystemConstants.BLINK_ON_TIME));
    pattern.applyTo(viewRight);
    addressableLED.setData(buffer);
  }

  public void setMiddleColor(Color color, boolean blink) {
    LEDPattern pattern = LEDPattern.solid(color);
    if(blink)
      pattern.blink(Seconds.of(LEDSubsystemConstants.BLINK_ON_TIME));
    pattern.applyTo(viewMiddle);
    addressableLED.setData(buffer);
  }

  public void setAll(Color color, boolean blink) {
    setLeftColor(color, blink);
    setRightColor(color, blink);
    setMiddleColor(color, blink);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
