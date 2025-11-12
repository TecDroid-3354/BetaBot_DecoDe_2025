package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.IMU
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.commands.JoystickCmd
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SolversMecanum


@TeleOp(name = "CMD", group = "Op Mode")
class CMDOpMode : CommandOpMode() {
    //lateinit var mecanum: Mecanum
    lateinit var mecanum: SolversMecanum
    lateinit var gamepadEx: GamepadEx
    lateinit var imu: IMU
    lateinit var otos: SparkFunOTOS
    lateinit var revHubOrientation: RevHubOrientationOnRobot
    override fun initialize() {
        imu = hardwareMap.get(IMU::class.java, "imu")
        imu.resetYaw()

        revHubOrientation = RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )

        imu.initialize(IMU.Parameters(revHubOrientation))
        otos = hardwareMap.get(SparkFunOTOS::class.java, "otos")

        //mecanum = Mecanum(hardwareMap, telemetry)
        mecanum = SolversMecanum(hardwareMap, telemetry)
        mecanum.defaultCommand = JoystickCmd(
            { (-gamepad1.left_stick_x).toDouble() },
            { (gamepad1.left_stick_y).toDouble() },
            { (-gamepad1.right_stick_x).toDouble() },
            { getRobotYaw(AngleUnit.DEGREES) },
            mecanum
        )

        gamepadEx = GamepadEx(gamepad1)

        configureButtonBindings()

    }

    fun getRobotYaw(angleUnit: AngleUnit): Double = imu.robotYawPitchRollAngles.getYaw(angleUnit)

    fun configureButtonBindings() {
        GamepadButton(gamepadEx, GamepadKeys.Button.START)
            .whenPressed(InstantCommand({
                imu.resetYaw()
            }))
    }

    fun periodic() {
        /*telemetry.addData("frontRight", mecanum.frontRightMotor.getLinearVelocity().mps)
        telemetry.addData("frontLeft", mecanum.frontLeftMotor.getLinearVelocity().mps)
        telemetry.addData("backRight", mecanum.backRightMotor.getLinearVelocity().mps)
        telemetry.addData("backLeft", mecanum.backLeftMotor.getLinearVelocity().mps)*/

        telemetry.addData("RobotYaw", getRobotYaw(AngleUnit.DEGREES))
        telemetry.addData("X", otos.position.x)
        telemetry.addData("Y", otos.position.y)
        telemetry.addData("H", otos.position.h)


        telemetry.update()
    }


    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize()

        waitForStart()

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            periodic()

            run()
        }
        reset()
    }
}