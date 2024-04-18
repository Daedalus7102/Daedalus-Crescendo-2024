// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;

import java.util.Random;


public class Leds extends SubsystemBase {
  
  public Leds() {m_led.setLength(m_ledBuffer.getLength()); m_led.start(); timer.start();}

  // -- CONFIG -- //
  private static final int HALF_LENGTH = 25; // La mitad de la cantidad de leds que hay
  private static final int PORT = 0; // El puerto PWM en el que se conectan los leds

  private AddressableLED m_led = new AddressableLED(PORT);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(HALF_LENGTH*2);
  private Color[] drawBuffer = new Color[HALF_LENGTH];
  private Timer timer = new Timer();
  private Random random = new Random();
  private int currentEffect = 0;
  private boolean isSerpentine = false;
  private int color = 0;

  /*  EFFECT IDs: 
  0 - Blowtorch
  1 - Pulse
  2 - Bounce
  3 - Breathe */
  public void setEffect(int effectID, int hsv_hue, boolean serpentine) {
    currentEffect = effectID;
    color = hsv_hue;
    isSerpentine = serpentine;
  }

  public void reset() {
    currentEffect = 0;
    color = 0;
    isSerpentine = false;
  }

  private void draw() {

    if (currentEffect == 0) {
      blowtorch();
    } else if (currentEffect == 1) {
      pulse();
    } else if (currentEffect == 2) {
      bounce();
    } else if (currentEffect == 3 ) {
      breathe();
    }

    for (int i = 0; i < HALF_LENGTH; i++) {
      drawBuffer[i] = m_ledBuffer.getLED(i);
    }

    if (isSerpentine) {
      for (int i = HALF_LENGTH; i < HALF_LENGTH*2; i++) {
        m_ledBuffer.setLED(i, drawBuffer[HALF_LENGTH*2 - i - 1]);
      }
      return;
    }
    for (int i = HALF_LENGTH; i < HALF_LENGTH*2; i++) {
      m_ledBuffer.setLED(i, drawBuffer[i-HALF_LENGTH]);
    }

  }

  // -- EFECTOS -- //
  private int[] heat = new int[HALF_LENGTH];
  private static final int COOLING = 100;
  private static final boolean REVERSE_DIRECTION = false;
  private void blowtorch() {
    // int buf = random.nextInt(30) + 250;
    
    for (int i = 0; i <= random.nextInt(2) + 1; i++){
      //heat[i] = buf;
      //heat[i] = random.nextInt(200, 255);
      heat[i] = 255;
    }

    for( int i = 0; i < HALF_LENGTH; i++) {
      heat[i] = heat[i] - random.nextInt(((COOLING * 10) / HALF_LENGTH) + 5);
      heat[i] = heat[i] < 0 ? 0 : heat[i];
    }
  
    for( int k= HALF_LENGTH - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }
  
    for( int j = 0; j < HALF_LENGTH; j++) {
      Color color = ColorUtils.heatColor( heat[j]);
      int pixelnumber;
      if ( REVERSE_DIRECTION ) {
        pixelnumber = (HALF_LENGTH-1) - j;
      } else {
        pixelnumber = j;
      }
      m_ledBuffer.setLED(pixelnumber, color);
    }
  }

  private void breathe() {
    double pos = Math.abs(Math.sin(timer.get() * 2 * Math.PI)); // Smoothly oscillates between 0 and 1
    int maxBrightness = 255;
    int minBrightness = 0;
    int centerIndex = 12;
    int threshold = 70; // Adjust the threshold to define the sharpness of the edge

    // Calculate brightness based on the position within the pulse cycle
    int brightness = (int) (minBrightness + pos * (maxBrightness - minBrightness));

    // Set the brightness for each LED
    for (int i = 0; i < HALF_LENGTH; i++) {
        int distanceFromCenter = Math.abs(centerIndex - i);
        int brightnessDelta = (int) (brightness * (1 - (double) distanceFromCenter / centerIndex));

        // Apply the step function for sharper edges
        if (brightnessDelta >= threshold) {
            m_ledBuffer.setHSV(i, color, 255, brightnessDelta); // Assuming RGB LED strip, adjust as needed
        } else {
            m_ledBuffer.setHSV(i, color, 255, minBrightness); // Set to minimum brightness below the threshold
        }
    }
}

  private void pulse() {
    double pos = Math.sin(timer.get() * 20);
    if (pos >= 0) {
      for (int i=0; i < HALF_LENGTH; i++) {
        m_ledBuffer.setHSV(i, color, 255, 255);
      }
    } else {
      for (int i=0; i < HALF_LENGTH; i++) {
        m_ledBuffer.setHSV(i, 0,0,0);
      }
    }
  }

  private void bounce() {
    float calc = HALF_LENGTH-1;

    int pos = (int) Math.round(Math.sin(timer.get() * 4) * calc/2 + calc/2);

    for (int i =0; i < HALF_LENGTH; i++){
      m_ledBuffer.setLED(i, Color.kBlack);
    }
    if (pos-2 >= 0) {
      m_ledBuffer.setHSV(pos-2, color, 255, 150);
    }
    if (pos-1 >= 0) {
      m_ledBuffer.setHSV(pos-1, color, 255, 200);
    }
    m_ledBuffer.setHSV(pos, color, 255, 255);
    if (pos+1 <= HALF_LENGTH) {
      m_ledBuffer.setHSV(pos+1, color, 255, 200);
    }
    if (pos+2 <= HALF_LENGTH) {
      m_ledBuffer.setHSV(pos+2, color, 255, 150);
    }
  } 

  private static class ColorUtils {
    private static Color heatColor(int temperature) {
      // Pura magia negra
      int t191 = scale32Video(temperature, 191);
  
      int heatramp = t191 & 0x3F; // 0..63
      heatramp <<= 2; // scale up to 0..252
      
      int[] buf = new int[3];
      if ((t191 & 0x80) != 0) {
          buf[0] = 255;
          buf[1] = 255;
          buf[2] = heatramp;
      } else if ((t191 & 0x40) != 0) {
          buf[0] = 255;
          buf[1] = heatramp;
          buf[2] = 0;
      } else {
          buf[0] = heatramp;
          buf[1] = 0;
          buf[2] = 0;
      }
      Color heatcolor = new Color(buf[0], buf[1], buf[2]);

      return heatcolor;
    }

    private static int scale32Video(int i, int scale) {
    // Pura magia negra
      int j;
   
      if (i != 0) {
          long result = (long) i * (long) scale;
          j = (int) (result / 255); // Divide by 255 to simulate the behavior of 8-bit integers
          if (scale != j) {
              j -= 255;
          }
      } else {
          j = 0;
      }
  
      return j;
    }    
  }

  @Override
  public void periodic() {
    draw();
    m_led.setData(m_ledBuffer);
  }
}
