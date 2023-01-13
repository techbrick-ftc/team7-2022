package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Autonomous(name = "ArmCycle", group = "Auto")
public class ArmCycle extends StarterAuto {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        double armDrop = 0.749;
        double stringDrop = 0.974;

        double armPicks [] = {2.041,2.105,2.145,2.152,2.23};
        double stringPicks [] = {0.641,0.644,0.642,0.649,0.658};

        boolean armDone0 = false;
        boolean stringDone0 = false;

        boolean armDoneFirst = false;

        while (opModeIsActive() && !armDoneFirst) {
            armDoneFirst = armAsync(armDrop + 0.25, false, 1);
        }

        while(opModeIsActive() && (!armDone0 || !stringDone0)){
            armDone0 = armAsync(armDrop, true, 1);
            stringDone0 = stringAsync(stringDrop);
        }
        grabbaOpen();

        for (int cone = 0; cone<5; cone++) {

            //Grab Align
            boolean armDone = false;
            boolean stringDone = false;
            wristServo.setPosition(0);
            while (opModeIsActive() && (!armDone || !stringDone)) {
                armDone = armAsync(armPicks[cone] - 0.5, false, 1);
                stringDone = stringAsync(stringPicks[cone]);
            }

            //Grab
            boolean armDone2 = false;
            grabbaOpen();
            while (opModeIsActive() && !armDone2) {
                armDone2 = armAsync(armPicks[cone], true, 1);
            }
            sleep(200);
            grabbaClose();
            sleep(200);


            boolean armDone3 = false;

            while (opModeIsActive() && !armDone3) {
                armDone3 = armAsync(armDrop + 0.6, false, 1);
            }
            wristServo.setPosition(0.94);

            boolean armDone4 = false;
            boolean stringDone4 = false;

            while (opModeIsActive() && (!armDone4 || !stringDone4)) {
                armDone4 = armAsync(armDrop, true, 1);
                stringDone4 = stringAsync(stringDrop);
            }
            armMotor.setPower(0);
            stringMotor.setPower(0);
            grabbaOpen();
            sleep(200);
        }
    }
}
