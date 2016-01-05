package com.qualcomm.ftcrobotcontroller.opmodes.OurPrograms;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


public class DistanceTest extends OpMode {

    OpticalDistanceSensor distanceSensor;
    @Override
    public void init() {

        distanceSensor = hardwareMap.opticalDistanceSensor.get("dist1");
    }
    @Override
    public void loop()
    {
        double lineSensor = distanceSensor.getLightDetected();
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", lineSensor));
    }

}
