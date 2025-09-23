package org.firstinspires.ftc.teamcode.subsystems.drivetrain

import LinearVelocity
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import com.seattlesolvers.solverslib.geometry.Translation2d
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveKinematics
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.velocityMotorEx.VelocityMotorConfig
import org.firstinspires.ftc.teamcode.utils.velocityMotorEx.VelocityMotorEx
import kotlin.math.abs
import kotlin.math.sign


class Mecanum(val hw: HardwareMap, val telemetry: Telemetry) : SubsystemBase() {

    lateinit var frontRightMotor: VelocityMotorEx
    lateinit var frontLeftMotor: VelocityMotorEx
    lateinit var backRightMotor: VelocityMotorEx
    lateinit var backLeftMotor: VelocityMotorEx

    private val frPIDFCoefficients = PIDFCoefficients(2.0, 0.0, 0.0, 15.5)
    private val flPIDFCoefficients = PIDFCoefficients(2.0, 0.0, 0.0, 16.0)
    private val brPIDFCoefficients = PIDFCoefficients(3.0, 0.0, 0.0, 18.5)
    private val blPIDFCoefficients = PIDFCoefficients(2.0, 0.0, 0.0, 17.5)



    // Kinematic
    // Locations of the wheels relative to the robot center.
    var frontLeftLocation: Translation2d = Translation2d(0.381, 0.381)
    var frontRightLocation: Translation2d = Translation2d(0.381, -0.381)
    var backLeftLocation: Translation2d = Translation2d(-0.381, 0.381)
    var backRightLocation: Translation2d = Translation2d(-0.381, -0.381)

    // Creating my kinematics object using the wheel locations.
    var kinematics: MecanumDriveKinematics = MecanumDriveKinematics(
        frontLeftLocation, frontRightLocation,
        backLeftLocation, backRightLocation
    )

    var prevVelLeft = 0.0
    var prevVelRight = 0.0

    var maxDelta = 0.05

    init {
        motorsConfig()

    }

    fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds) {
        val wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds)

        val flVel = smoothVel(prevVelLeft, wheelSpeeds.frontLeftMetersPerSecond);
        val blVel = smoothVel(prevVelLeft, wheelSpeeds.rearLeftMetersPerSecond);
        val frVel = smoothVel(prevVelRight, wheelSpeeds.frontRightMetersPerSecond);
        val brVel = smoothVel(prevVelRight, wheelSpeeds.rearRightMetersPerSecond);

        prevVelLeft  = (flVel + blVel) / 2.0;
        prevVelRight  = (frVel + brVel) / 2.0;

        frontRightMotor.setVelocity(LinearVelocity.fromMps(frVel))
        frontLeftMotor.setVelocity(LinearVelocity.fromMps(flVel))
        backRightMotor.setVelocity(LinearVelocity.fromMps(brVel))
        backLeftMotor.setVelocity(LinearVelocity.fromMps(blVel))
    }

    // Función para limitar la aceleración
    private fun smoothVel(prev: Double, target: Double): Double {
        val delta = target - prev
        if (abs(delta) > maxDelta) {
            return prev + sign(delta) * maxDelta
        }
        return target
    }
    fun motorsConfig() {
        frontRightMotor = VelocityMotorEx(hw.get(DcMotorEx::class.java, MecanumConstants.Ids.frId),
            VelocityMotorConfig(
                DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotorSimple.Direction.REVERSE,
                MecanumConstants.Physics.ticksPerRevolution,
                frPIDFCoefficients
            ))

        frontLeftMotor = VelocityMotorEx(hw.get(DcMotorEx::class.java, MecanumConstants.Ids.flId),
            VelocityMotorConfig(
                DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotorSimple.Direction.REVERSE,
                MecanumConstants.Physics.ticksPerRevolution,
                flPIDFCoefficients
            ))

        backRightMotor = VelocityMotorEx(hw.get(DcMotorEx::class.java, MecanumConstants.Ids.brId),
            VelocityMotorConfig(
                DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotorSimple.Direction.FORWARD,
                MecanumConstants.Physics.ticksPerRevolution,
                brPIDFCoefficients
            ))

        backLeftMotor = VelocityMotorEx(hw.get(DcMotorEx::class.java, MecanumConstants.Ids.blId),
            VelocityMotorConfig(
                DcMotor.ZeroPowerBehavior.FLOAT,
                DcMotorSimple.Direction.REVERSE,
                MecanumConstants.Physics.ticksPerRevolution,
                blPIDFCoefficients
            ))

        frontRightMotor.setCircumference(MecanumConstants.Physics.circumference)
        frontLeftMotor.setCircumference(MecanumConstants.Physics.circumference)
        backRightMotor.setCircumference(MecanumConstants.Physics.circumference)
        backLeftMotor.setCircumference(MecanumConstants.Physics.circumference)

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS)
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS)
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS)
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS)
    }
}