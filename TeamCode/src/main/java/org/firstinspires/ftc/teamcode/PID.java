package org.firstinspires.ftc.teamcode;




public class PID<T> {
    interface RetValue<T> {
        void retValue(T value);
    }
    private T Pgain, IGain, Dgain;

    public void PIDController(T Pgain, T IGain, T Dgain, T ControlInput, RetValue<T> ReturnValue) {

    }
}
