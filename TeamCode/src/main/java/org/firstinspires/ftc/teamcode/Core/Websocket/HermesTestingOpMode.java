package org.firstinspires.ftc.teamcode.Core.Websocket;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Hermes Testing", group="Testing")
public class HermesTestingOpMode extends OpMode
{
    HermesLog log;
    RobotPose poseToSend;

    @Override
    public void init() {
        log = new HermesLog();
        log.Init("HERMES_TESTING", 500, this);
    }

    @Override
    public void start() {
        log.Start();
    }

    @Override
    public void loop() {
        poseToSend = new RobotPose(Math.random(), Math.random(), Math.random());
        Object[] data = {poseToSend};
        log.AddData(data);
        log.Update();
    }
}