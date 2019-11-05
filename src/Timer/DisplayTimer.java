package Timer;
import java.lang.*;
import javafx.animation.AnimationTimer;

public class DisplayTimer extends AnimationTimer {

    private String timerText ="0.00";

    private long startTime;
    private long pauseDuration = 0;
    private long lastPauseTime;
    private long lastResumeTime;

    @Override
    public void start() {
        pauseDuration = 0;
        startTime = System.currentTimeMillis();
        super.start();
    }

    public void pause() {
        super.stop();
        lastPauseTime = System.currentTimeMillis();
    }

    public void resume() {
        super.start();
        lastResumeTime = System.currentTimeMillis();
        pauseDuration += lastResumeTime - lastPauseTime ;
    }

    @Override
    public void handle(long timestamp) {
        long now = System.currentTimeMillis();
        double duration = (now - startTime - pauseDuration) / 1000.0;
        timerText = String.format("%.2f", duration);
    }

    public String getTimerLbl() {
        return this.timerText;
    }

    public void initialize() {
        timerText = "0.00";
    }
}
