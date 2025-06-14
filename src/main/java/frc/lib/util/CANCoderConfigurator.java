package frc.lib.util;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

public class CANCoderConfigurator {
    private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5,
            java.util.concurrent.TimeUnit.MILLISECONDS, queue);

    public void applyConfig(CANcoder cancoder, CANcoderConfiguration config) {
        threadPoolExecutor.submit(() -> {
            for (int i = 0; i < 5; i++) {
                StatusCode result = cancoder.getConfigurator().apply(config);
                if (result.isOK()) {
                    break;
                }
            }
        });
    }
}
