package frc.robot.Autos.Common;

import edu.wpi.first.wpilibj.Timer;

public abstract class FastAutoBase {

    public abstract void routine() throws Exception;
    protected boolean m_active = false;

    public void run() {
        m_active = true;
        try {
            routine();
        } catch (Exception e) {
            System.out.println("Auto mode done, ended early");
            return;
        }
        System.out.println("Auto mode done");
    }
    public void stop() {
        m_active = false;
    }

    public boolean isActive() {
        return m_active;
    }

    public boolean isActiveWithThrow() throws Exception {
        if (!isActive()) {
            throw new Exception();
        }

        return isActive();
    }

    public void runCommand(FastCommand command) throws Exception{
        isActiveWithThrow();
        command.init();

        while(isActiveWithThrow() && !command.isFinished()){
            command.run();
            Timer.delay(0.02);
        }

        command.end();
    }

}
