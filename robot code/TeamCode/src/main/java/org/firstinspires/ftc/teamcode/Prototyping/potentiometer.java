package org.firstinspires.ftc.teamcode.Prototyping;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name="potentiometer",group = "Tests")
public class potentiometer  extends OpMode{
    private AnalogInput potentiometer = null;
    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public void init() {
        potentiometer = hardwareMap.get(AnalogInput.class,"potentiometer");
        telemetry.addLine("robot pos: initilazied");
    }

    @Override
    public void loop() {
        telemetry.addData("potentiometer pos: " , potentiometer.getVoltage() / 3.347 * 270);
        telemetry.update();
    }
}
