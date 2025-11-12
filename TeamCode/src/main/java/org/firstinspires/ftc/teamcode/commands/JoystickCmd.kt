package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.CommandBase
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import com.seattlesolvers.solverslib.util.MathUtils.clamp
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Mecanum
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumConstants
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SolversMecanum
import java.util.function.DoubleSupplier
import kotlin.math.pow


/**
 * Creates a new DefaultDrive.
 *
 * @param mecanum The drive subsystem this command will run on.
 * @param x x joystick value
 * @param y y joystick value
 * @param rx rx joystick value
 */

class JoystickCmd(
    val x: DoubleSupplier,
    val y: DoubleSupplier,
    val rx: DoubleSupplier,
    val yawInDegrees: DoubleSupplier,
    val mecanum: SolversMecanum
) : CommandBase() {

    init {
        addRequirements(mecanum)
    }

    override fun execute() {

        val yVel = clamp(y.asDouble.pow(3.0), -1.0, 1.0) //* MecanumConstants.Velocities.maxVelocityY.mps
        val xVel = clamp(x.asDouble.pow(3.0), -1.0, 1.0) //* MecanumConstants.Velocities.maxVelocityX.mps
        val rxVel = clamp(rx.asDouble.pow(3.0), -1.0, 1.0) //* MecanumConstants.Velocities.maxRotationVelocity.radPerSec

        val speeds = ChassisSpeeds(
            yVel,
            xVel,
            rxVel)

        //mecanum.setChassisSpeeds(speeds)
        mecanum.setChassisSpeedsFromFieldOriented(speeds, yawInDegrees.asDouble)
    }
}