package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "McProtoNugget", group = "Testing")
//@Disabled
public class McProtoNugget extends OpMode {

    // Declare OpMode members.
    private DcMotorEx topShooter = null;
    private DcMotorEx bottomShooter = null;

    private final double LAUNCHER_TARGET_VELOCITY = 2000;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        topShooter = hardwareMap.get(DcMotorEx.class, "top_shooter");
        bottomShooter = hardwareMap.get(DcMotorEx.class, "bottom_shooter");

        // Shooter configs
        topShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        topShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        bottomShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        topShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        launch();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void launch() {
        if (gamepad1.right_bumper) {
            topShooter.setVelocity(LAUNCHER_TARGET_VELOCITY);
            bottomShooter.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            topShooter.setVelocity(0);
            bottomShooter.setVelocity(0);
        }
    }
}