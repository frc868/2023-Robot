package frc.robot.subsystems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Modes;
import frc.robot.GamePieceLocation.GamePiece;
import frc.robot.Modes.RobotState;

/**
 * The LEDs subsystem, which controls the state of the LEDs through an internal
 * state machine and continuously updates the LED's buffer.
 * 
 * @author dr
 */
@LoggedObject
public class LEDs extends SubsystemBase {
    /** The LEDs. */
    private AddressableLED leds = new AddressableLED(0);

    /** The current state of the LEDs. */
    @Log
    private LEDState state = LEDState.Uninitialized;

    @Log
    private Supplier<Boolean> isError = () -> this.state == LEDState.Error;

    /**
     * Describes the states that the LEDs can be in.
     */
    public enum LEDState {
        /** Rainbow pattern that moves across the strip. */
        Rainbow(LEDState::rainbow),
        /** TechHOUNDS pattern, which uses the official colors and . */
        TechHOUNDS(LEDState::techHounds),
        /** Illuminated yellow to signify the cone pickup mode. */
        ConePickup(LEDState::conePickup),
        /** Illuminated yellow to signify the cone pickup mode. */
        ConePickupFlashing(LEDState::conePickupFlashing),
        /** Illuminated purple to signify the cube pickup mode. */
        CubePickup(LEDState::cubePickup),
        /** Illuminated purple to signify the cube pickup mode. */
        CubePickupFlashing(LEDState::cubePickupFlashing),
        /** Blinking red to signify an error state. */
        Error(LEDState::error),
        /**
         * Solid red, will only be turned on while an input is commanding the bot to do
         * a disallowed action.
         */
        TemporaryError(LEDState::temporaryError),
        /** Blinking red to signify an error state. */
        Uninitialized(LEDState::uninitialized);

        /**
         * The hue of the first pixel in the strand, used to "move" patterns down the
         * strand.
         */
        private static int firstPixelHue = 0;
        /** The common buffer object used between each buffer supplier. */
        private static AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LEDs.LENGTH);
        /** The current time in 20ms ticks, used for the error state. */
        private static int timeStep = 0;

        /** The supplier for the buffer for each LEDState. */
        private final Runnable bufferRunnable;

        private static final int v = 255;

        /** The ctor for the LEDState, takes in a Supplier<AddressableLEDBuffer>. */
        private LEDState(Runnable bufferRunnable) {
            this.bufferRunnable = bufferRunnable;
        }

        /**
         * Get the buffer for this LEDState.
         * 
         * @return the buffer
         */
        public AddressableLEDBuffer getBuffer() {
            this.bufferRunnable.run();
            return buffer;
        }

        /**
         * Changes the contents of the AddressableLEDBuffer to the rainbow state.
         */
        private static void rainbow() {
            for (int i = 0; i < buffer.getLength(); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to precess
                final var hue = (firstPixelHue + (i * 180 / buffer.getLength())) % 180;
                // Set the value
                buffer.setHSV(i, hue, 255, v);

            }
            // Increase by 3 to make the rainbow "move"
            firstPixelHue += 3;
            // Check bounds
            firstPixelHue %= 180;
        }

        /**
         * Changes the contents of the AddressableLEDBuffer to the TechHOUNDS state.
         */
        private static void techHounds() {
            for (int i = 0; i < buffer.getLength(); i++) {
                if (((i + timeStep) / 8) % 2 == 0) { // shifts back and forth every 8 pixels
                    buffer.setHSV(i, 15, 250, v); // gold
                } else {
                    buffer.setHSV(i, 109, 240, v); // blue
                }
            }
            timeStep += 1;
        }

        /**
         * Changes the contents of the AddressableLEDBuffer to the cone pickup state.
         */
        private static void conePickup() {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setHSV(i, 15, 255, v);
            }
        }

        /**
         * Changes the contents of the AddressableLEDBuffer to the cone pickup flashing
         * state.
         */
        private static void conePickupFlashing() {
            timeStep++; // 50 timesteps is one second
            timeStep %= 10; // every 1 second it will roll over
            for (int i = 0; i < buffer.getLength(); i++) {
                if (timeStep / 5 == 1) {
                    buffer.setHSV(i, 15, 255, v);
                } else {
                    buffer.setHSV(i, 0, 0, 0);
                }
            }
        }

        /**
         * Changes the contents of the AddressableLEDBuffer to the cube pickup state.
         */
        private static void cubePickup() {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setHSV(i, 150, 255, v);
            }
        }

        /**
         * Changes the contents of the AddressableLEDBuffer to the cube pickup flashing
         * state.
         */
        private static void cubePickupFlashing() {
            timeStep++; // 50 timesteps is one second
            timeStep %= 10; // every 1 second it will roll over
            for (int i = 0; i < buffer.getLength(); i++) {
                if (timeStep / 5 == 1) {
                    buffer.setHSV(i, 150, 255, v);
                } else {
                    buffer.setHSV(i, 0, 0, 0);
                }
            }
        }

        /**
         * Changes the contents of the AddressableLEDBuffer to the error state.
         */
        private static void error() {
            timeStep++; // 50 timesteps is one second
            timeStep %= 50; // every 1 second it will roll over
            for (int i = 0; i < buffer.getLength(); i++) {
                // every 0.5 seconds it will switch from off to on
                if (timeStep / 25 == 1) {
                    buffer.setHSV(i, 0, 255, v);
                } else {
                    buffer.setHSV(i, 0, 0, 0);
                }
            }
        }

        /**
         * Changes the contents of the AddressableLEDBuffer to the temporary error
         * state.
         */
        private static void temporaryError() {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setHSV(i, 0, 255, v);
            }
        }

        /**
         * Changes the contents of the AddressableLEDBuffer to the uninitialized state
         * (orange).
         */
        private static void uninitialized() {
            timeStep++; // 50 timesteps is one second
            timeStep %= 50; // every 0.4 seconds it will roll over
            for (int i = 0; i < buffer.getLength(); i++) {
                // every 0.2 seconds it will switch from off to on
                if (timeStep / 25 == 1) {
                    buffer.setHSV(i, 4, 255, v);
                } else {
                    buffer.setHSV(i, 0, 0, 0);
                }
            }
        }
    }

    /**
     * Initializes the LEDs.
     */
    public LEDs() {
        leds.setLength(50);
        leds.setData(state.getBuffer());
        leds.start();

        setDefaultCommand(updateStateMachineCommand());
    }

    /**
     * Gets the current state of the LEDs.
     * 
     * @return the current LEDState.
     */
    public LEDState getLEDState() {
        return state;
    }

    /**
     * Sets the state of the LEDs, and sets the previousState to the old current
     * state.
     * 
     * @param state the new LEDState.
     */
    public void setLEDState(LEDState state) {
        this.state = state;
    }

    /**
     * Creates a command that changes the state of the LEDs, then idles.
     * 
     * @param state the new LEDState.
     * @return the command
     */
    public Command holdLEDStateCommand(Supplier<LEDState> state) {
        return runOnce(() -> this.setLEDState(state.get())).andThen(Commands.run(() -> {
        })).ignoringDisable(true).withName("Hold LED State");
    }

    public Command updateStateMachineCommand() {
        return run(() -> {
            if (Modes.getIntakeMode().isPresent()) {
                if (Modes.getIntakeMode().get() == GamePiece.CONE) {
                    if (Modes.getRobotState() == RobotState.INTAKING)
                        setLEDState(LEDState.ConePickupFlashing);
                    else
                        setLEDState(LEDState.ConePickup);
                } else if (Modes.getIntakeMode().get() == GamePiece.CUBE) {
                    if (Modes.getRobotState() == RobotState.INTAKING)
                        setLEDState(LEDState.CubePickupFlashing);
                    else
                        setLEDState(LEDState.CubePickup);
                }
            } else {
                setLEDState(LEDState.TechHOUNDS);
            }

            if (DriverStation.isDisabled()) {
                setLEDState(LEDState.Rainbow);
            }

            if (!Modes.isInitialized()) {
                setLEDState(LEDState.Uninitialized);
            }

            if (Modes.getCurrentDiscreteError().isPresent()) {
                setLEDState(LEDState.Error);
            }
            if (Modes.getCurrentContinuousError().isPresent()) {
                setLEDState(LEDState.TemporaryError);
            }
        })
                .ignoringDisable(true)
                .withName("Update State Machine");
    }

    /**
     * Sets the LEDs to the current state of the buffer.
     */
    @Override
    public void periodic() {
        super.periodic();
        leds.setData(state.getBuffer());
    }
}
