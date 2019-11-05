package Robot;

public enum Command {

    FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT, SEND_SENSORS, ALIGN_FRONT, ALIGN_RIGHT, INITIAL_CALIBRATE, ALIGN_FRONT1, FINAL_CALIBRATE;

    public enum ArduinoMove {
        W, S, A, D, K, O, P, T, F, Z;
    }
}
