package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class teleopsolo extends OpMode {
    final int lift_high = 2950;
    final int lift_mid = 2200;
    final int lift_low = 1400;
    final int turret_center = 0;
    final int turret_right = 360;
    final int turret_left = -360;
    final double claw_open = 0.8;
    final double claw_close = 0;

    private DistanceSensor distance;
    private PIDController slidescontroller;
    public static double ps = 0.002, is = 0, ds = 0;
    public static double fs = 0.01;
    public static int targets = 0;
    private final double ticks_in_degrees = 384 / 360.0;
    private DcMotorEx slidesleft;
    private DcMotorEx slidesright;
    private Servo turret;
    ElapsedTime liftTimer = new ElapsedTime();




    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightRear;
    DcMotor leftRear;

    Servo claw;
    public static int currentposleft;
    public static int currentposright;
    public static int currentposturret;
    public enum LiftState{
        LIFTSTART,
        LIFTCLAWCLOSE,
        LIFTEXTEND,
        LIFTTURN,
        LIFTDROP,
        LIFTARMRESET,
        LIFTRETRACT,
        LIFTRETRACTED,


    }
    LiftState liftstate = LiftState.LIFTSTART;
    @Override
    public void init() {
        liftTimer.reset();
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        leftRear = hardwareMap.dcMotor.get("leftRear");

        slidescontroller = new PIDController(ps, is, ds);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slidesleft = hardwareMap.get(DcMotorEx.class, "slidesLeft");
        slidesright = hardwareMap.get(DcMotorEx.class, "slidesRight");

        slidesleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidesright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesright.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.servo.get("claw");
        turret = hardwareMap.servo.get("turret");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setPosition(0.6);
        turret.setPosition(0.5);
        distance = hardwareMap.get(DistanceSensor.class, "distance");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distance;



    }
    @Override
    public void loop() {
        switch (liftstate) {
            case LIFTSTART:
                if (distance.getDistance(DistanceUnit.INCH) < 5) {
                    claw.setPosition(1);
                    liftTimer.reset();
                    liftstate = LiftState.LIFTCLAWCLOSE;

                }
                break;

            case LIFTCLAWCLOSE:
                if (liftTimer.seconds()>=0.5) {
                    targets = 2950;
                    liftstate = LiftState.LIFTEXTEND;
                }
                break;
            case LIFTEXTEND:
                if (slidesleft.getCurrentPosition() > 1400 && slidesright.getCurrentPosition() > 1400) {
                    turret.setPosition(0.1);
                    liftstate = LiftState.LIFTTURN;
                }
                break;
            case LIFTTURN:
                if (slidesleft.getCurrentPosition() - 2950 < 10 && slidesright.getCurrentPosition() - 2950 < 10 && turret.getPosition() == 0.1) {
                    liftstate = LiftState.LIFTDROP;
                }
                break;
            case LIFTDROP:
                if (gamepad1.x) {
                    claw.setPosition(0.6);
                    liftTimer.reset();
                    liftstate = LiftState.LIFTARMRESET;
                }
                break;
            case LIFTARMRESET:
                if (liftTimer.seconds() >= 0.5) {
                    turret.setPosition(0.5);
                   liftTimer.reset();
                    liftstate = LiftState.LIFTRETRACT;
                }
                break;
            case LIFTRETRACT:
                if(turret.getPosition()==0.5&&liftTimer.seconds()>=0.5){
                    targets= 0;
                    liftstate = LiftState.LIFTRETRACTED;
                }
            case LIFTRETRACTED:
                if (slidesleft.getCurrentPosition() < 10 && slidesright.getCurrentPosition() < 10 ) {
                    liftstate = LiftState.LIFTSTART;
                }
                break;
            default:
                liftstate = LiftState.LIFTSTART;


        }
        if (gamepad1.y && liftstate != LiftState.LIFTSTART) {
            liftstate = LiftState.LIFTSTART;
        }


        slidescontroller.setPID(ps, is, ds);
        int slidesleftpos = slidesleft.getCurrentPosition();
        double pidleft = slidescontroller.calculate(slidesleftpos, targets);
        double ffleft = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

        int slidesrightpos = slidesright.getCurrentPosition();
        double pidright = slidescontroller.calculate(slidesrightpos, targets);
        double ffright = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

        double powerleft = pidleft + ffleft;

        double powerright = pidright + ffright;

        slidesleft.setPower(powerleft);
        slidesright.setPower(powerright);

        telemetry.addData("posslides", slidesleftpos);
        telemetry.addData("target", targets);


        currentposleft = slidesleft.getCurrentPosition();
        currentposright = slidesright.getCurrentPosition();

        telemetry.addData("Left Spool", currentposleft);
        telemetry.addData("Right Spool", currentposright);

        telemetry.update();


        if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
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


        leftFront.setPower(frontLeftPower / denominator);
        leftRear.setPower(backLeftPower / denominator);
        rightFront.setPower(frontRightPower / denominator);
        rightRear.setPower(backRightPower / denominator);

    }
}
