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

        double armDrop = 0.772;
        double stringDrop = 0.936;

        double armPick1 = 2.064;
        double stringPick1 = 0.641;

        double armPick2 = 2.105;
        double stringPick2 = 0.644;

        double armPick3 = 2.145;
        double stringPick3 = 0.642;

        double armPick4 = 2.152;
        double stringPick4 = 0.649;

        double armPick5 = 2.23;
        double stringPick5 = 0.658;

        //Grab Align
        armAsync(armPick1 - 0.5);
        stringAsync(stringPick1);

        //Grab
        grabbaServo.setPosition(0.3);
        armSync(armPick1);
        sleep(200);
        grabbaServo.setPosition(1);
        sleep(200);
        wristServo.setPosition(0.94);

        //Align
        armAsync(armDrop + 0.3);

        //Release
        stringSync(stringDrop);
        armSync(armDrop);
        sleep(200);
        grabbaServo.setPosition(0.3);
        sleep(200);
        armSync(1);
        wristServo.setPosition(0);




    }
}
