// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.util.Random;

public class Leds extends SubsystemBase {
  
  private static final int LENGTH = 50; // La cantidad de leds que hay (Total)
  private static final int PORT = 0; // El puerto PWM en el que se conectan los leds

  private AddressableLED m_led = new AddressableLED(PORT);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LENGTH);
  private int m_rainbowFirstPixelHue = 0;

  private Random random = new Random();

  // algo para el fuego
  private int[] heat = new int[50];
  private static final int COOLING = 30;
  private static final int SPARKING = 90;
  private static final boolean REVERSE_DIRECTION = false;

  public Leds() {m_led.setLength(LENGTH); m_led.start();}

  private static int[] heatColor(int temperature) {
    // Pura magia negra
    int t191 = scale32Video(temperature, 191);

    int heatramp = t191 & 0x3F; // 0..63
    heatramp <<= 2; // scale up to 0..252

    int[] heatcolor = new int[3];

    if ((t191 & 0x80) != 0) {
        heatcolor[0] = 255;
        heatcolor[1] = 255;
        heatcolor[2] = heatramp;
    } else if ((t191 & 0x40) != 0) {
        heatcolor[0] = 255;
        heatcolor[1] = heatramp;
        heatcolor[2] = 0;
    } else {
        heatcolor[0] = heatramp;
        heatcolor[1] = 0;
        heatcolor[2] = 0;
    }
    return heatcolor;
  }

  private static int scale32Video(int i, int scale) {
  // No tengo idea que esta pasando aqui pero funciona
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

  private void rainbow() {
    // funcion bien pinchi gei :v
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;
  }

  private void blowtorch() {
    for( int i = 0; i < LENGTH; i++) {
      heat[i] = heat[i] -  random.nextInt(0, ((COOLING * 10) / LENGTH) + 2);
    }
  
    for( int k= LENGTH - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }
  
    if( random.nextInt(255) < SPARKING ) {
      int y = random.nextInt(7);
      heat[y] = heat[y] + random.nextInt(160,255);
    }
  
    for( int j = 0; j < LENGTH; j++) {
      int[] color = heatColor( heat[j]);
      int pixelnumber;
      if ( REVERSE_DIRECTION ) {
        pixelnumber = (LENGTH-1) - j;
      } else {
        pixelnumber = j;
      }
      m_ledBuffer.setRGB(pixelnumber, color[0], color[1], color[2]);
    }

  }

  @Override
  public void periodic() {
    blowtorch();
    m_led.setData(m_ledBuffer);
  }
}
