package org.firstinspires.ftc.teamcode.Reyansh.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import dev.nextftc.core.subsystems.Subsystem;

public class ColorSensor implements Subsystem {
    public static ColorSensor INSTANCE = new ColorSensor();
    RevColorSensorV3 colorsensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
    Servo rgbindicator = hardwareMap.get(Servo.class, "rgbindicator");

    public void init(HardwareMap hardwareMap) {
        // initialization logic (runs on init)
        RevColorSensorV3 colorsensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        Servo rgbindicator = hardwareMap.get(Servo.class, "rgbindicator");

    }
    float[] hsvValues = new float[3];
    NormalizedRGBA colors = colorsensor.getNormalizedColors();
    double hue = JavaUtil.colorToHue(colors.toColor()); // <------ New code
    double sat = JavaUtil.colorToSaturation(colors.toColor()); // <------ New code
    double val = JavaUtil.colorToValue(colors.toColor()); // <------ New code

    public ElapsedTime runtime = new ElapsedTime();
    double artifactcounter = 0;

    public void IncountBalls(){
        if (hue > 167 && runtime.milliseconds() > 200) {
            artifactcounter +=1;
            runtime.reset();
        }
        if (sat > 0.5 && runtime.milliseconds() > 200) {
            artifactcounter +=1;
            runtime.reset();
        }
    }

    public void light() {
        if (artifactcounter == 0) {
            rgbindicator.setPosition(0);
        } else if (artifactcounter == 1) {
            rgbindicator.setPosition(0.3);
        } else if (artifactcounter == 2) {
            rgbindicator.setPosition(0.375);
        } else if (artifactcounter == 3) {
            rgbindicator.setPosition(0.5);
        } else {
            rgbindicator.setPosition(0.6);
        }
    }
    @Override
    public void periodic() {
        // periodic logic (runs every loop)
        light();

    }
}
