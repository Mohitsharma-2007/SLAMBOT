package com.slambot.sensorhub;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RadialGradient;
import android.graphics.Shader;
import android.util.AttributeSet;
import android.view.View;
import android.animation.ValueAnimator;
import java.util.Random;

public class EyesView extends View {
    public static final int STATE_IDLE = 0;
    public static final int STATE_MOVING = 1;
    public static final int STATE_WARNING = 2;
    public static final int STATE_COLLISION = 3;

    private int mState = STATE_IDLE;
    private float mBlinkFactor = 1.0f; // 1.0 = fully open, 0.0 = closed
    private float mPupilSizeFactor = 1.0f;
    private float mEyeScaleX = 1.0f;
    private float mEyeScaleY = 1.0f;

    private Paint mEyePaint;
    private Paint mGlowPaint;
    private ValueAnimator mBlinkAnimator;
    private ValueAnimator mBreathingAnimator;
    private Random mRandom = new Random();

    public EyesView(Context context) {
        super(context);
        init();
    }

    public EyesView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        mEyePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        mEyePaint.setStyle(Paint.Style.FILL);

        mGlowPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        mGlowPaint.setStyle(Paint.Style.FILL);

        // Breathing animation for glowing eyes
        mBreathingAnimator = ValueAnimator.ofFloat(0.9f, 1.1f);
        mBreathingAnimator.setDuration(1500);
        mBreathingAnimator.setRepeatMode(ValueAnimator.REVERSE);
        mBreathingAnimator.setRepeatCount(ValueAnimator.INFINITE);
        mBreathingAnimator.addUpdateListener(animation -> {
            mPupilSizeFactor = (float) animation.getAnimatedValue();
            invalidate();
        });
        mBreathingAnimator.start();

        // Start random blink schedule
        postDelayed(new Runnable() {
            @Override
            public void run() {
                triggerBlink();
                postDelayed(this, 3000 + mRandom.nextInt(4000));
            }
        }, 3000);
    }

    public void setRobotState(int state) {
        if (mState != state) {
            mState = state;
            invalidate();
        }
    }

    private void triggerBlink() {
        if (mBlinkAnimator != null && mBlinkAnimator.isRunning()) {
            return;
        }
        mBlinkAnimator = ValueAnimator.ofFloat(1.0f, 0.0f, 1.0f);
        mBlinkAnimator.setDuration(250);
        mBlinkAnimator.addUpdateListener(animation -> {
            mBlinkFactor = (float) animation.getAnimatedValue();
            invalidate();
        });
        mBlinkAnimator.start();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        int width = getWidth();
        int height = getHeight();
        if (width == 0 || height == 0) return;

        int eyeColor = Color.CYAN;
        int glowColor = Color.parseColor("#3300FFFF"); // semi-transparent cyan

        switch (mState) {
            case STATE_IDLE:
                eyeColor = Color.parseColor("#00E5FF"); // Bright Cyan
                glowColor = Color.parseColor("#3300E5FF");
                break;
            case STATE_MOVING:
                eyeColor = Color.parseColor("#00E676"); // Vibrant Green
                glowColor = Color.parseColor("#3300E676");
                break;
            case STATE_WARNING:
                eyeColor = Color.parseColor("#FFD600"); // Neon Yellow
                glowColor = Color.parseColor("#33FFD600");
                break;
            case STATE_COLLISION:
                eyeColor = Color.parseColor("#FF1744"); // Glowing Red
                glowColor = Color.parseColor("#33FF1744");
                break;
        }

        float centerX = width / 2f;
        float centerY = height / 2f;
        float eyeSpacing = width * 0.25f; // Horizontal separation
        float eyeRadiusX = width * 0.12f;
        float eyeRadiusY = height * 0.18f;

        // Draw Left Eye
        drawSingleEye(canvas, centerX - eyeSpacing, centerY, eyeRadiusX, eyeRadiusY, eyeColor, glowColor);

        // Draw Right Eye
        drawSingleEye(canvas, centerX + eyeSpacing, centerY, eyeRadiusX, eyeRadiusY, eyeColor, glowColor);
    }

    private void drawSingleEye(Canvas canvas, float cx, float cy, float rx, float ry, int eyeColor, int glowColor) {
        float activeRy = ry * mBlinkFactor * mEyeScaleY;
        float activeRx = rx * mEyeScaleX;

        if (activeRy < 1f) return; // Completely shut

        // Radial glow effect
        RadialGradient glowShader = new RadialGradient(
            cx, cy, rx * 2.5f,
            new int[]{glowColor, Color.TRANSPARENT},
            null, Shader.TileMode.CLAMP
        );
        mGlowPaint.setShader(glowShader);
        canvas.drawCircle(cx, cy, rx * 2.5f, mGlowPaint);

        // Inner solid core
        RadialGradient eyeShader = new RadialGradient(
            cx, cy, rx * mPupilSizeFactor,
            new int[]{Color.WHITE, eyeColor, Color.parseColor("#05050A")},
            new float[]{0.0f, 0.7f, 1.0f}, Shader.TileMode.CLAMP
        );
        mEyePaint.setShader(eyeShader);

        // Draw the main eye shape
        canvas.drawOval(cx - activeRx, cy - activeRy, cx + activeRx, cy + activeRy, mEyePaint);
    }
}
