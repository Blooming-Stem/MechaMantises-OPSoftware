package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOP extends OpMode {
    final int lift_high = 2950;
    final int lift_mid = 2200;
    final int lift_low = 1400;
    final int turret_center = 0;
    final int turret_right = 360;
    final int turret_left = -360;
    final double claw_open = 0.8;
    final double claw_close = 0;
    private PIDController controller;
    public static double p = 0.002, i = 0, d = 0.0001;
    public static double f = 0.01;
    public static int target = 0;
    private final double ticks_in_degree = 1440 / 360.0;
    private DcMotorEx arm_motor;

    private PIDController slidescontroller;
    public static double ps = 0.002, is = 0, ds = 0;
    public static double fs = 0.01;
    public static int targets = 0;
    private final double ticks_in_degrees = 384 / 360.0;
    private DcMotorEx slidesleft;
    private DcMotorEx slidesright;



    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightRear;
    DcMotor leftRear;

    Servo claw;
    public static int currentposleft;
    public static int currentposright;
    public static int currentposturret;
    public enum LIFTSTATE{
        LIFTUP,
        LIFTIDLE,


    }
    LIFTSTATE liftstate = LIFTSTATE.LIFTIDLE;
    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        leftRear = hardwareMap.dcMotor.get("leftRear");

        slidescontroller = new PIDController(ps, is, ds);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slidesleft = hardwareMap.get(DcMotorEx.class, "slidesleft");
        slidesright = hardwareMap.get(DcMotorEx.class, "slidesright");

        slidesleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidesright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesright.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.servo.get("claw");
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "turret");
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setPosition(0);


    }
    @Override
    public void loop() {

        if(gamepad2.left_bumper){
            switch (liftstate){
                case LIFTIDLE:
                    
            }
        }

        slidescontroller.setPID(ps, is, ds);
        int slidesleftpos = slidesleft.getCurrentPosition();
        double pidleft = slidescontroller.calculate(slidesleftpos, targets);
        double ffleft = Math.cos(Math.toRadians(targets/ticks_in_degrees))* fs;

        int slidesrightpos = slidesright.getCurrentPosition();
        double pidright = slidescontroller.calculate(slidesrightpos, targets);
        double ffright = Math.cos(Math.toRadians(targets/ticks_in_degrees))* fs;

        double powerleft = pidleft + ffleft;

        double powerright = pidright + ffright;

        slidesleft.setPower(powerleft);
        slidesright.setPower(powerright);

        telemetry.addData("posslides", slidesleftpos);
        telemetry.addData("target", targets);

        controller.setPID(p, i, d);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree))* f;

        double power = pid + ff;

        arm_motor.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);

        currentposleft = slidesleft.getCurrentPosition();
        currentposright = slidesright.getCurrentPosition();

        telemetry.addData("Left Spool", currentposleft);
        telemetry.addData("Right Spool", currentposright);
        telemetry.addData("Turret", currentposturret);
        telemetry.update();

        if (gamepad1.x){
            claw.setPosition(0.8);
        }else if (gamepad1.b){
            claw.setPosition(0);
        }

        if(gamepad2.y) {

            targets = 2950;

        }else if(gamepad2.a) {
            targets = 0;

        }
        else if(gamepad2.dpad_up){
            targets = 2200;
        }
        else if(gamepad2.dpad_right){
            targets = 1400;
        }
        if(gamepad2.x){
            if(slidesleft.getCurrentPosition()>50||slidesright.getCurrentPosition()>50) {
                target = -360;
            }
        }else if(gamepad2.b){
            if(slidesleft.getCurrentPosition()>50||slidesright.getCurrentPosition()>50) {
                target = 360;
            }

        }
        else if(gamepad2.right_bumper){
            target = 0;
        }
        if(gamepad1.left_stick_y ==0&&gamepad1.left_stick_x==0){
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);

        }

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

    }
}
