// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private int fade = 0;
  private int direction = 0;
  private AddressableLED m_led0 = new AddressableLED(0);
  private AddressableLEDBuffer m_ledBuffer0 = new AddressableLEDBuffer(864);

  private int i = 0;
  public int time = 0;
  private int mode = 0;
  public boolean isEnabled = false;
  public boolean intakeNote = false;
  public boolean shootReady = false;
  public boolean climbTime = false;

  private int m_rainbowFirstPixelHue;

  public LED() {
    m_led0.setLength(m_ledBuffer0.getLength());
    m_led0.start();

   }

  @Override
  public void periodic() {
    if (isEnabled == false) {
      //rainbow();
     //setLedFade();
         for (var i = 0; i < m_ledBuffer0.getLength(); i = i + 3) {
        m_ledBuffer0.setRGB(i, 0, 0, 0);
         }
      m_led0.setData(m_ledBuffer0);
    } else{
      if(climbTime == true){
        rainbow();
        m_led0.setData(m_ledBuffer0);
      } else if(intakeNote){

      }
    }
  }


  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer0.getLength(); i = i + 9) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer0.getLength())) % 180;
      // Set the value
      m_ledBuffer0.setHSV(i, hue, 255, 128);

    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }


  public void setLedFade() {
    if(time>-1) {
      time ++;
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      for (i = 0; i < m_ledBuffer0.getLength(); i++) {
        m_ledBuffer0.setHSV(i, 80, 255, fade);
      }
      if (direction == 0) {
        fade += 4;
        if (fade == 128) {
          direction = 1;
        }
      } else if (direction == 1) {
        fade -= 4;
        if (fade == 0) {
          direction = 0;
        }
      }
    } else {
      for (i = 0; i < m_ledBuffer0.getLength(); i++) {
        m_ledBuffer0.setHSV(i, 0, 255, fade);
      }
      if (direction == 0) {
        fade += 4;
        if (fade == 128) {
          direction = 1;
        }
      } else if (direction == 1) {
        fade -= 4;
        if (fade == 0) {
          direction = 0;
        }
      }
    }
  } else {
      time ++;
      direction = 1;
      if(direction == 1){
        i++;
        m_ledBuffer0.setHSV(i,0,255,255);
        m_ledBuffer0.setHSV(i-1,0,255,0);
      } else {
        i--;
        m_ledBuffer0.setHSV(i,0,255,255);
        m_ledBuffer0.setHSV(i+1,0,255,0);
      }
      if(i > 36){
        direction = 0;
      } else if(i<3){
        direction = 1;
      }

  }
    m_led0.setData(m_ledBuffer0);
  }



  public void setMode(int mode) {
    this.mode = mode;
  }

}