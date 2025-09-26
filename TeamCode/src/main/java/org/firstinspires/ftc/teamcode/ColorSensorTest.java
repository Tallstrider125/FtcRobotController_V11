package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@TeleOp(name="ColorSensorTest", group="ZTEST")
public class ColorSensorTest extends OpMode{

    public RevColorSensorV3 colorSensor = null;
     //public  NormalizedColorSensor colorSensor;

    @Override
public void init(){
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "CS");
        telemetry.addData(">","RobotReady.   Press Play.");
        telemetry.update();
    }
    @Override
    public void init_loop() {
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(false);
        }
    }
    @Override
    public void start() {
    }
    @Override
    public void loop() {
        runColorTest();

    }
    public void runColorTest(){
            // read all 3 color channels in one I2C transmission:
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            telemetry.addData("Red: ", colors.red);
            telemetry.addData("Blue: ", colors.blue);
            telemetry.addData("Green ", colors.green);
            telemetry.update();

    }


    public void stop(){

    }
}
