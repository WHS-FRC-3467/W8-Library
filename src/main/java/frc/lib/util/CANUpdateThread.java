package frc.lib.util;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.Supplier;
import com.ctre.phoenix6.StatusCode;

public class CANUpdateThread {
    // Executor for retrying config operations asynchronously
    private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5,
        java.util.concurrent.TimeUnit.MILLISECONDS, queue);

    /**
     * Attempts a CTRE action up to 5 times until it succeeds.
     *
     * @param action The status-returning operation to retry.
     */
    public void CTRECheckErrorAndRetry(Supplier<StatusCode> action)
    {
        threadPoolExecutor.submit(() -> {
            for (int i = 0; i < 5; i++) {
                StatusCode result = action.get();
                if (result.isOK()) {
                    break;
                }
            }
        });
    }
}
