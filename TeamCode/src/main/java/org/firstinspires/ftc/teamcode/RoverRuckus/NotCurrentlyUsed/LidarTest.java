package org.firstinspires.ftc.teamcode.RoverRuckus.NotCurrentlyUsed;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoverRuckus.RoverHardware;

@TeleOp(name = "Lidar Test", group = "Teleop")
@Disabled
public class LidarTest extends LinearOpMode {
    RoverHardware robot = new RoverHardware();

    DistanceSensor lidar;

    public void runOpMode(){
        robot.init(hardwareMap);

        lidar = hardwareMap.get(DistanceSensor.class, "lidar");

        telemetry.addData(">>", "Press Start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("deviceName",lidar.getDeviceName() );
            telemetry.addData("range", String.format("%.01f cm", lidar.getDistance(DistanceUnit.CM)));
            telemetry.update();
            idle();
        }
    }
}
