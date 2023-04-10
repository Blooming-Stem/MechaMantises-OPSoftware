package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
@TeleOp
//@Disabled
public class odotest extends OpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public Encoder leftEncoder;
    public Encoder rightEncoder;
    public Encoder frontEncoder;
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightFront;
    public DcMotor rightRear;
    @Override
    public void init() {


        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));

    }

    @Override
    public void loop() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power
        // (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx);
        double backLeftPower = (y - x + rx);
        double frontRightPower = (y - x - rx);
        double backRightPower = (y + x - rx);



        leftFront.setPower(frontLeftPower/denominator);
        leftRear.setPower(backLeftPower/denominator);
        rightFront.setPower(frontRightPower/denominator);
        rightRear.setPower(backRightPower/denominator);
        telemetry.addData("Left Encoder", leftEncoder.getCorrectedVelocity());
        telemetry.addData("Right Encoder", rightEncoder.getCorrectedVelocity());
        telemetry.addData("Front Encoder", frontEncoder.getCorrectedVelocity());
        telemetry.update();



    }
}
