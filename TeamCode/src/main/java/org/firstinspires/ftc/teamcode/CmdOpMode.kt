package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.commands.JoystickCmd
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.FeedforwardMecanum

@TeleOp(name = "CMD", group = "Op Mode")
class CMDOpMode : CommandOpMode() {
    //lateinit var mecanum: Mecanum
    lateinit var mecanum: FeedforwardMecanum
    lateinit var gamepadEx: GamepadEx
    override fun initialize() {
        //mecanum = Mecanum(hardwareMap, telemetry)
        mecanum = FeedforwardMecanum(hardwareMap, telemetry, gamepad1)
        mecanum.defaultCommand = JoystickCmd(
            { (-gamepad1.left_stick_x).toDouble() },
            { (-gamepad1.left_stick_y).toDouble() },
            { (-gamepad1.right_stick_x).toDouble() },
                mecanum
        )

        gamepadEx = GamepadEx(gamepad1)

        configureButtonBindings()

    }

    fun configureButtonBindings() {

    }

    fun periodic() {
        telemetry.addData("frontRight", mecanum.frontRightMotor.getLinearVelocity().mps)
        telemetry.addData("frontLeft", mecanum.frontLeftMotor.getLinearVelocity().mps)
        telemetry.addData("backRight", mecanum.backRightMotor.getLinearVelocity().mps)
        telemetry.addData("backLeft", mecanum.backLeftMotor.getLinearVelocity().mps)

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