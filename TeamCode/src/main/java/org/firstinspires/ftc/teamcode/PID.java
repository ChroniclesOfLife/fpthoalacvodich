package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;


public class PID {
    private float _Pgain, _Igain, _Dgain, _Target = 0;
    private float _CycIcomp;
    private float _Icomp;
    private float _CycDcomp;
    private float _Dcomp;
    private float _Error;
    private float _LastError;
    private float _CurrTime;
    private float _LastTime;
    private float _DeltaTime = 0;
    private float _MaxCumulation = 30000.0F;
    private ElapsedTime Time;

    public void PIDController(float Pgain, float Igain, float Dgain, float ControlInput, Out ReturnValue) {
        _Pgain = Pgain;
        _Igain = Igain;
        _Dgain = Dgain;

        float _Pcomp = ControlInput - _Target;

        _CurrTime = Time.time(TimeUnit.MILLISECONDS);

        _DeltaTime = _CurrTime - _LastTime;

        _CycIcomp = (_Error + _LastError) / 2 * _DeltaTime;

        _Icomp += _CycIcomp;

        _CycDcomp = (_Error - _LastError) / _DeltaTime;

        if (_Icomp > _MaxCumulation) _Icomp = _MaxCumulation;
        if (_Icomp < -_MaxCumulation) _Icomp = -_MaxCumulation;

        ReturnValue.PIDout((_Pcomp * _Pgain) + (_Icomp * _Igain) + (_Dcomp * _Dgain));
    }

    public void SetTarget(float Target) {
        _Target = Target;
    }

    public void SetPGain(float Pgain) {
        _Pgain = Pgain;
    }

    public void SetIGain(float Igain) {
        _Igain = Igain;
    }

    public void SetPDain(float Dgain) {
        _Dgain = Dgain;
    }

    interface Out {
        public void PIDout(float value);
    }
}
