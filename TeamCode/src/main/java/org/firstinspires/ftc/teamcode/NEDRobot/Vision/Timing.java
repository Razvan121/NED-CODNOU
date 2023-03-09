package org.firstinspires.ftc.teamcode.NEDRobot.Vision;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Timing {

    public static class Timer{
        private ElapsedTime time;
        private long timerLength;
        private long pauseTime;
        private TimeUnit unit;
        private boolean timerOn;

        public Timer(long timerLength, TimeUnit unit){
            this.timerLength = timerLength;
            this.unit = unit;
            this.time = new ElapsedTime();
            time.reset();
        }



        public Timer(long timerLength){
            this(timerLength,TimeUnit.MILLISECONDS);
        }

        public void start(){
            time.reset();
            pauseTime = 0;
            timerOn = true;
        }

        public void pause(){
            if(timerOn){
                pauseTime = time.nanoseconds();
                timerOn = false;
            }
        }

    public void resume(){
            if(!timerOn){
                time = new ElapsedTime(System.nanoTime() - pauseTime);
                timerOn = true;
            }
    }

    public long elapsedTime(){
            if(timerOn){
                return time.time(unit);
            }
            else
                return  unit.convert(pauseTime,TimeUnit.NANOSECONDS);

    }

    public long remainingTime(){
            return timerLength - elapsedTime();
    }

    public boolean done(){
            return elapsedTime() >= timerLength;
    }

    public boolean isTimerOn(){
            return timerOn;
    }

    }

    public class Rate {

        private ElapsedTime time;
        private long rate;

        public Rate(long rateMillis) {
            rate = rateMillis;
            time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }

        public void reset() {
            time.reset();
        }

        public boolean atTime() {
            boolean done = (time.milliseconds() >= rate);
            time.reset();
            return done;
        }

    }
}
