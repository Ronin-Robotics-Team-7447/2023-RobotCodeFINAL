package com.team6560.frc2023.subsystems;

import org.w3c.dom.css.RGBColor;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    private AddressableLED led;
    private AddressableLEDBuffer ledbuffer;
    String mode;

    public Lights() {
        led = new AddressableLED(0);
        ledbuffer = new AddressableLEDBuffer(6);
        led.setLength(ledbuffer.getLength());
        setLightsToDefault();
    }

    public void setLightsToDefault() {
        mode = "default";
        ledbuffer.setRGB(0, 41, 202, 254 );
        ledbuffer.setRGB(1, 255, 255, 255 );
        ledbuffer.setRGB(2, 41, 202, 254 );
        ledbuffer.setRGB(3, 255, 255, 255 );
        ledbuffer.setRGB(4, 41, 202, 254 );
        ledbuffer.setRGB(5, 255, 255, 255 );

        led.setData(ledbuffer);
        led.start();
    }

    public void setLightsToCone() {
        mode = "cone";
        for( int i = 0; i < ledbuffer.getLength(); i++ ) {
            ledbuffer.setRGB(i, 255, 255, 0);
        }
        led.setData(ledbuffer);
        led.start();
    }

    public void setLightsToCube() {
        mode = "cube";
        for( int i = 0; i < ledbuffer.getLength(); i++ ) {
            ledbuffer.setRGB(i, 128, 0, 128);
        }
        led.setData(ledbuffer);
        led.start();
    }

    public String getMode() {
        return mode;
    }

    public void setLights(int red, int green, int blue) {
        for( int i = 0; i < ledbuffer.getLength(); i++ ) {
            ledbuffer.setRGB(i, red, green, blue);
        }
        led.setData(ledbuffer);
        led.start();
    }
}
