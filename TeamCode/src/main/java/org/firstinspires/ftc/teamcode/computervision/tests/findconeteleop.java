package org.firstinspires.ftc.teamcode.computervision.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.computervision.CVMaster;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.AxisDirection;
import org.firstinspires.ftc.teamcode.roadrunner.util.BNO055IMUUtil;

@TeleOp
@Config
public class findconeteleop extends LinearOpMode {
    BNO055IMU imu;

    private PIDController pidController;
    public static double kp = 0.0175, ki = 0, kd = 0.000;
    private double pid = 0;


    public void runOpMode(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_X);
        imu.initialize(parameters);
        pidController = new PIDController(kp, ki, kd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        CVMaster cv = new CVMaster(this);
        cv.observeStick();
        while(!isStarted()&&!isStopRequested()) {
            if (cv.pipeline.pixel_height > 0) {
                telemetry.addLine("Ready!");

            } else {


            }

            telemetry.update();

        }

        while(opModeIsActive()){
            if(cv.pipeline.getDistanceFromCone()<50){
                telemetry.addData("Distance", cv.pipeline.getDistanceFromCone());
                pidController.setPID(kp, ki, kd);
                pid = pidController.calculate(imu.getAngularOrientation().firstAngle, cv.pipeline.turnAngle());





            }
            else{
                telemetry.addLine("No cone in sight");
                pid = 0;
            }
            drive.setMotorPowers(pid, pid, -pid, -pid);








            telemetry.update();
        }
    }
}
