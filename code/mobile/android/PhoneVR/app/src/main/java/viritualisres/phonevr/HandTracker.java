/* (C)2026 */
package viritualisres.phonevr;

import android.content.Context;
import android.graphics.Bitmap;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.util.Log;
import com.google.mediapipe.framework.image.BitmapImageBuilder;
import com.google.mediapipe.framework.image.MPImage;
import com.google.mediapipe.tasks.components.containers.Category;
import com.google.mediapipe.tasks.components.containers.Landmark;
import com.google.mediapipe.tasks.components.containers.NormalizedLandmark;
import com.google.mediapipe.tasks.core.BaseOptions;
import com.google.mediapipe.tasks.core.Delegate;
import com.google.mediapipe.tasks.vision.core.RunningMode;
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarker;
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarkerResult;
import java.nio.ByteBuffer;
import java.util.List;

/**
 * Finds hands in the pictures ARCore already takes, on its own thread so it can never hold up head
 * tracking or the stream. A picture arrives only when the last one is finished. See hand_geometry.h
 * for what happens to the results.
 */
final class HandTracker {
    private static final String TAG = "PhoneVR6DoF";
    private static final String MODEL = "hand_landmarker.task";

    // Wrist first, then the finger bases: the fingertips are wrapped around a Joy-Con.
    private static final int[] PALM = {0, 1, 5, 9, 13, 17};
    private static final int VALUES_PER_HAND = 2 + PALM.length * 5;
    private static final int MAX_HANDS = 2;
    private static final float MIN_CONFIDENCE = 0.4f; // a gripping hand scores lower than a bare one

    private final Context context;
    private final float[] values = new float[VALUES_PER_HAND * MAX_HANDS];
    private HandlerThread thread;
    private Handler handler;
    private HandLandmarker landmarker;
    private Bitmap bitmap;

    HandTracker(Context context) {
        this.context = context.getApplicationContext();
    }

    void start() {
        thread = new HandlerThread("hands");
        thread.start();
        handler = new Handler(thread.getLooper());
        handler.post(this::load);
    }

    void stop() {
        Handler h = handler;
        handler = null;
        h.post(
                () -> {
                    if (landmarker != null) landmarker.close();
                    landmarker = null;
                });
        thread.quitSafely();
    }

    private void load() {
        // GPU first: on the CPU the model made head tracking and the picture visibly jitter
        try {
            landmarker = create(Delegate.GPU);
            Log.i(TAG, "hands: model ready on the GPU");
            return;
        } catch (RuntimeException e) {
            Log.w(TAG, "hands: GPU not available, using the CPU", e);
        }
        try {
            landmarker = create(Delegate.CPU);
            Log.i(TAG, "hands: model ready on the CPU");
        } catch (RuntimeException e) {
            Log.e(TAG, "hands: model failed to load", e);
            problemNative("The phone couldn't load the hand model: " + e.getMessage());
        }
    }

    private HandLandmarker create(Delegate delegate) {
        BaseOptions base =
                BaseOptions.builder().setModelAssetPath(MODEL).setDelegate(delegate).build();
        return HandLandmarker.createFromOptions(
                context,
                HandLandmarker.HandLandmarkerOptions.builder()
                        .setBaseOptions(base)
                        .setRunningMode(RunningMode.VIDEO)
                        .setNumHands(MAX_HANDS)
                        .setMinHandDetectionConfidence(MIN_CONFIDENCE)
                        .setMinHandPresenceConfidence(MIN_CONFIDENCE)
                        .setMinTrackingConfidence(MIN_CONFIDENCE)
                        .build());
    }

    // From native code, on ARCore's thread. `rgba` isn't touched again until resultNative.
    void onPicture(long timestampNs, int width, int height, ByteBuffer rgba) {
        Handler h = handler;
        if (h != null) h.post(() -> check(timestampNs, width, height, rgba));
    }

    private void check(long timestampNs, int width, int height, ByteBuffer rgba) {
        if (landmarker == null) {
            resultNative(timestampNs, values, 0, 0);
            return;
        }
        long start = SystemClock.elapsedRealtimeNanos();
        int count = 0;
        try {
            if (bitmap == null || bitmap.getWidth() != width || bitmap.getHeight() != height) {
                bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
            }
            rgba.rewind();
            bitmap.copyPixelsFromBuffer(rgba);
            MPImage picture = new BitmapImageBuilder(bitmap).build();
            count = copyHands(landmarker.detectForVideo(picture, timestampNs / 1000000L));
        } catch (RuntimeException e) {
            Log.e(TAG, "hands: check failed", e);
            problemNative("Hand check failed: " + e.getMessage());
        }
        float ms = (SystemClock.elapsedRealtimeNanos() - start) / 1e6f;
        resultNative(timestampNs, values, count, ms);
    }

    private int copyHands(HandLandmarkerResult result) {
        int count = Math.min(result.landmarks().size(), MAX_HANDS);
        for (int hand = 0; hand < count; hand++) {
            List<NormalizedLandmark> picture = result.landmarks().get(hand);
            List<Landmark> world = result.worldLandmarks().get(hand);
            Category side = result.handedness().get(hand).get(0);
            int v = hand * VALUES_PER_HAND;
            values[v++] = "Left".equals(side.categoryName()) ? 1 : 0;
            values[v++] = side.score();
            for (int point : PALM) {
                values[v++] = picture.get(point).x();
                values[v++] = picture.get(point).y();
                values[v++] = world.get(point).x();
                values[v++] = world.get(point).y();
                values[v++] = world.get(point).z();
            }
        }
        return count;
    }

    private native void resultNative(long timestampNs, float[] values, int count, float msPerCheck);

    private native void problemNative(String problem);
}
