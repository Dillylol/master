package org.firstinspires.ftc.teamcode.jules.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import android.content.Context;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.jules.JulesRamTx;
import org.firstinspires.ftc.teamcode.jules.JulesTap;
import org.firstinspires.ftc.teamcode.jules.Metrics;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBuffer;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesHttpBridge;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesMetricsHttpAdapter;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesTokenStore;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;

import java.io.IOException;

@TeleOp(name="JULES Pedro Pathing", group="Jules")
public class JulesPedroPathingOpMode extends OpMode {
    DcMotorEx leftFront, rightFront, leftRear, rightRear;
    IMU imu;
    VoltageSensor vs;

    JulesTap tap;
    JulesRamTx tx;
    Follower follower;

    private JulesBuffer buffer;
    private JulesStreamBus streamBus;
    private JulesHttpBridge http;

    double lastEmit = 0;
    static final double PERIOD_S = 0.02;

    @Override public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class,"leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class,"rightRear");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        vs = hardwareMap.voltageSensor.iterator().next();

        follower = Constants.createFollower(hardwareMap);
        tap = new JulesTap(
                537.7, // Replace with your motor's TPR
                Constants.WHEEL_RADIUS * 2,
                Constants.GEAR_RATIO,
                vs,
                leftFront, rightFront, leftRear, rightRear);

        TelemetryManager panelsTM = PanelsTelemetry.INSTANCE.getTelemetry();
        tx = new JulesRamTx(50_000, panelsTM, telemetry, "jules");

        buffer    = new JulesBuffer(4096);
        streamBus = new JulesStreamBus();
        JulesMetricsHttpAdapter adapter = new JulesMetricsHttpAdapter(buffer);

        Context ctx = hardwareMap.appContext;
        String token = JulesTokenStore.getOrCreate(ctx);

        try {
            http = new JulesHttpBridge(58080, adapter, adapter, token, streamBus);
            telemetry.addData("JULES", "HTTP up");
        } catch (IOException e) {
            telemetry.addData("JULES HTTP", "failed: %s", e.getMessage());
        }
    }

    @Override public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        follower.setTeleOpDrive(y,x,rx, true);
        follower.update();

        double now = getRuntime();
        if (now - lastEmit >= PERIOD_S) {
            lastEmit = now;
            Metrics m = createMetrics();
            tap.setLastCmd(y); // Using forward power as the command power for simplicity

            if (http != null) telemetry.addLine(http.advertiseLine());

            tx.send(m);
            buffer.push(m);
            streamBus.publishJsonLine(JulesMetricsHttpAdapter.encodePublic(m));
        }

        telemetry.update();
    }


    @Override public void stop() {
        try { tx.close(); } catch (Exception ignored) {}
        if (http != null) http.close();
        if (streamBus != null) streamBus.close();
    }

    private Metrics createMetrics() {
        Metrics m = new Metrics();
        m.t = getRuntime();
        m.batteryV = vs.getVoltage();

        // Odometry
        m.x = follower.getPose().getX();
        m.y = follower.getPose().getY();
        m.heading = follower.getPose().getHeading();

        // IMU
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        m.headingDeg = orientation.getYaw(AngleUnit.DEGREES);
        m.pitch = orientation.getPitch(AngleUnit.DEGREES);
        m.roll = orientation.getRoll(AngleUnit.DEGREES);
        m.yawRate = angularVelocity.zRotationRate;
        m.pitchRate = angularVelocity.xRotationRate;
        m.rollRate = angularVelocity.yRotationRate;

        return m;
    }
}