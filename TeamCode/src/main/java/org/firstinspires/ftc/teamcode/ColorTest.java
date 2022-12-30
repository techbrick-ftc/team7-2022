package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp

public class ColorTest extends StarterAuto {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
        public void runOpMode ()
        {
            initialize();
            waitForStart();

            packet.put("encoder", backRight.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);

            double startAngle = imu.getAngularOrientation().firstAngle;

            motorsStop();


            while (opModeIsActive()) {
                telemetry.addData("red FL", colorFL.red());
                telemetry.addData("red BR", colorBR.red());
                telemetry.addData("red FR", colorFR.red());
                telemetry.addData("red BL", colorBL.red());


                telemetry.addData("blue FL", colorFL.blue());
                telemetry.addData("blue BR", colorBR.blue());
                telemetry.addData("blue FR", colorFR.blue());
                telemetry.addData("blue BL", colorBL.blue());
                telemetry.update();
            }
        }
    }
