package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

@Autonomous
public class SimpleAuto extends StarterAuto {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        initialize();
        initAprilTags();
        packet.addLine("id before");
        dashboard.sendTelemetryPacket(packet);



        waitForStart();
        packet.addLine("id after");
        dashboard.sendTelemetryPacket(packet);
        int tag = getAprilTag();
        /*
        if (tag == 1){
            double startAngle = imu.getAngularOrientation().firstAngle;

            int frontpos = frontRight.getCurrentPosition();
            //Continue moving until robot is on tape
            while (opModeIsActive() && colorFR.red() < 400 && frontRight.getCurrentPosition() - frontpos < 5500) {
                drivingCorrectionLeft(startAngle, 0.5);
                //  packet.addLine("Red Color " + color1.red());
            }

            backLeft.setPower(0);
            frontRight.setPower(0);
            sleep(1000);

            double startAngle2 = imu.getAngularOrientation().firstAngle;

            backRight.setPower(0.5);
            frontLeft.setPower(0.5);


            double startTime = getRuntime();
            //Continue moving until robot is on tape and make sure robot has moved for at least 1 second
            // this is so robot is for sure off the tape
            int leftpos = frontLeft.getCurrentPosition();

            while (opModeIsActive() && ((colorFR.red() < 400 && frontLeft.getCurrentPosition() - leftpos < 4500) || getRuntime() - startTime < 1 )) {
                drivingCorrectionStraight(startAngle2, 0.5);
            }

            backRight.setPower(0);
            frontLeft.setPower(0);
            sleep(1000);

        }
        else if (tag == 2){
            double startAngle = imu.getAngularOrientation().firstAngle;

            int frontpos = frontRight.getCurrentPosition();
            //Continue moving until robot is on tape
            while (opModeIsActive() && colorFR.red() < 400 && frontRight.getCurrentPosition() - frontpos < 5500) {
                drivingCorrectionLeft(startAngle, 0.5);
                //  packet.addLine("Red Color " + color1.red());
            }

            backLeft.setPower(0);
            frontRight.setPower(0);
            sleep(1000);

            double startAngle2 = imu.getAngularOrientation().firstAngle;

            backRight.setPower(0.5);
            frontLeft.setPower(0.5);


            double startTime = getRuntime();
            //Continue moving until robot is on tape and make sure robot has moved for at least 1 second
            // this is so robot is for sure off the tape
            int leftpos = frontLeft.getCurrentPosition();

            while (opModeIsActive() && ((colorFR.red() < 400 && frontLeft.getCurrentPosition() - leftpos < 4500) || getRuntime() - startTime < 1 )) {
                drivingCorrectionStraight(startAngle2, 0.5);
            }

            backRight.setPower(0);
            frontLeft.setPower(0);
            sleep(1000);

            frontRight.setPower(-0.5);
            backLeft.setPower(0.5);
            sleep(1000);

        }
        else if (tag == 3){
            double startAngle3 = imu.getAngularOrientation().firstAngle;
            frontRight.setPower(-0.5);
            backLeft.setPower(0.5);
            sleep(1000);

            drivingCorrectionStraight(startAngle3, 0.5);
            //drive
        }
            */
    }

}
