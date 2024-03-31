package frc.robot.Common;

public class FastTeleopLambda extends FastTeleOpAction{
    public interface voidInterface {
        void f();
    }
    voidInterface mf;

    public FastTeleopLambda(voidInterface mf){
        this.mf = mf;
    }
    @Override
    public void run(){
        mf.f();
    }
}
