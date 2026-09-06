package com.slambot.sensorhub;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;
import androidx.annotation.Nullable;

import org.json.JSONArray;
import org.json.JSONObject;

public class LiveMapView extends View {
    private Paint mGridPaint;
    private Paint mRobotPaint;
    private Paint mRobotGlowPaint;
    private Paint mTextPaint;
    private Paint mTrailPaint;
    private Bitmap mMapBitmap;

    private int mMapWidth = 0;
    private int mMapHeight = 0;
    private float mResolution = 0.05f; // meters per pixel
    private float mOriginX = 0f;
    private float mOriginY = 0f;

    private float mRobotX = 0f; // in meters
    private float mRobotY = 0f; // in meters
    private float mRobotTheta = 0f; // in radians

    private float mScale = 1.0f;
    private float mTranslateX = 0f;
    private float mTranslateY = 0f;
    private float mLastTouchX;
    private float mLastTouchY;
    private boolean mIsDragging = false;
    private ScaleGestureDetector mScaleDetector;

    public LiveMapView(Context context) {
        super(context);
        init(context);
    }

    public LiveMapView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    public LiveMapView(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init(context);
    }

    private void init(Context context) {
        mGridPaint = new Paint();
        mGridPaint.setColor(Color.parseColor("#151f30"));
        mGridPaint.setStrokeWidth(1.5f);

        mRobotPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        mRobotPaint.setColor(Color.parseColor("#00E5FF"));
        mRobotPaint.setStyle(Paint.Style.FILL);

        mRobotGlowPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        mRobotGlowPaint.setColor(Color.parseColor("#4400E5FF"));
        mRobotGlowPaint.setStyle(Paint.Style.FILL);

        mTrailPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        mTrailPaint.setColor(Color.parseColor("#00E676"));
        mTrailPaint.setStrokeWidth(3f);
        mTrailPaint.setStyle(Paint.Style.STROKE);

        mTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        mTextPaint.setColor(Color.parseColor("#8899B0"));
        mTextPaint.setTextSize(26f);

        mScaleDetector = new ScaleGestureDetector(context, new ScaleGestureDetector.SimpleOnScaleGestureListener() {
            @Override
            public boolean onScale(ScaleGestureDetector detector) {
                mScale *= detector.getScaleFactor();
                mScale = Math.max(0.2f, Math.min(mScale, 8.0f));
                invalidate();
                return true;
            }
        });
    }

    public void updateMap(JSONObject mapObj) {
        try {
            mMapWidth = mapObj.optInt("width", 0);
            mMapHeight = mapObj.optInt("height", 0);
            mResolution = (float) mapObj.optDouble("resolution", 0.05);

            JSONArray originArr = mapObj.optJSONArray("origin");
            if (originArr != null && originArr.length() >= 2) {
                mOriginX = (float) originArr.getDouble(0);
                mOriginY = (float) originArr.getDouble(1);
            }

            JSONArray dataArr = mapObj.optJSONArray("data");
            if (dataArr != null && mMapWidth > 0 && mMapHeight > 0) {
                int[] pixels = new int[mMapWidth * mMapHeight];
                for (int i = 0; i < pixels.length && i < dataArr.length(); i++) {
                    int val = dataArr.getInt(i);
                    if (val == -1) {
                        pixels[i] = Color.parseColor("#0A0E17"); // Unknown deep space
                    } else if (val == 0) {
                        pixels[i] = Color.parseColor("#152438"); // Free floor space
                    } else if (val >= 50) {
                        pixels[i] = Color.parseColor("#00E5FF"); // Solid obstacle / wall neon cyan
                    } else {
                        pixels[i] = Color.parseColor("#1A3654"); // Low confidence
                    }
                }

                Bitmap bmp = Bitmap.createBitmap(pixels, mMapWidth, mMapHeight, Bitmap.Config.ARGB_8888);
                mMapBitmap = bmp;
                postInvalidate();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void updateRobotPose(float x, float y, float theta) {
        mRobotX = x;
        mRobotY = y;
        mRobotTheta = theta;
        postInvalidate();
    }

    public void centerOnRobot() {
        if (mMapBitmap != null && getWidth() > 0 && getHeight() > 0) {
            float robotPixelX = (mRobotX - mOriginX) / mResolution;
            float robotPixelY = mMapHeight - ((mRobotY - mOriginY) / mResolution);

            mTranslateX = (getWidth() / 2f) - (robotPixelX * mScale);
            mTranslateY = (getHeight() / 2f) - (robotPixelY * mScale);
            invalidate();
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        mScaleDetector.onTouchEvent(event);

        switch (event.getActionMasked()) {
            case MotionEvent.ACTION_DOWN:
                mLastTouchX = event.getX();
                mLastTouchY = event.getY();
                mIsDragging = true;
                break;
            case MotionEvent.ACTION_MOVE:
                if (mIsDragging && !mScaleDetector.isInProgress()) {
                    float dx = event.getX() - mLastTouchX;
                    float dy = event.getY() - mLastTouchY;
                    mTranslateX += dx;
                    mTranslateY += dy;
                    mLastTouchX = event.getX();
                    mLastTouchY = event.getY();
                    invalidate();
                }
                break;
            case MotionEvent.ACTION_UP:
            case MotionEvent.ACTION_CANCEL:
                mIsDragging = false;
                break;
        }
        return true;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        int w = getWidth();
        int h = getHeight();

        // Background
        canvas.drawColor(Color.parseColor("#060910"));

        // Draw background grid lines
        float gridSize = 60f * mScale;
        if (gridSize > 15f) {
            float offsetX = mTranslateX % gridSize;
            float offsetY = mTranslateY % gridSize;
            for (float x = offsetX; x < w; x += gridSize) {
                canvas.drawLine(x, 0, x, h, mGridPaint);
            }
            for (float y = offsetY; y < h; y += gridSize) {
                canvas.drawLine(0, y, w, y, mGridPaint);
            }
        }

        canvas.save();
        canvas.translate(mTranslateX, mTranslateY);
        canvas.scale(mScale, mScale);

        // Draw Occupancy Grid Bitmap
        if (mMapBitmap != null) {
            canvas.drawBitmap(mMapBitmap, 0, 0, null);
        }

        // Draw Robot Pose & Heading Marker
        float robotPixelX = (mRobotX - mOriginX) / mResolution;
        float robotPixelY = mMapHeight > 0 ? (mMapHeight - ((mRobotY - mOriginY) / mResolution)) : 0;

        canvas.save();
        canvas.translate(robotPixelX, robotPixelY);
        canvas.rotate((float) Math.toDegrees(-mRobotTheta));

        // Glow ring
        canvas.drawCircle(0, 0, 16f / Math.max(mScale * 0.5f, 0.5f), mRobotGlowPaint);

        // Robot Directional Triangle
        Path path = new Path();
        float size = 12f / Math.max(mScale * 0.5f, 0.5f);
        path.moveTo(size * 1.4f, 0);
        path.lineTo(-size, -size * 0.9f);
        path.lineTo(-size * 0.4f, 0);
        path.lineTo(-size, size * 0.9f);
        path.close();

        canvas.drawPath(path, mRobotPaint);
        canvas.restore();

        canvas.restore();

        // HUD Overlay Text
        canvas.drawText(String.format("SLAM Map: %dx%d | Res: %.2fm", mMapWidth, mMapHeight, mResolution), 20, 40, mTextPaint);
        canvas.drawText(String.format("Pose: X=%.2f Y=%.2f θ=%.1f°", mRobotX, mRobotY, Math.toDegrees(mRobotTheta)), 20, 75, mTextPaint);
    }
}
