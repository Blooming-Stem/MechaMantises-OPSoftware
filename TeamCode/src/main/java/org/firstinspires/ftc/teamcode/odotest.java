package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;
@TeleOp
public class odotest extends OpMode {
    public Encoder leftEncoder;
    public Encoder rightEncoder;
    public Encoder frontEncoder;
    @Override
    public void init() {
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));

    }

    @Override
    public void loop() {
        telemetry.addData("Left ENcoder", leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
        telemetry.addData("Front Encoder", frontEncoder.getCurrentPosition());
        telemetry.update();

    }
}
