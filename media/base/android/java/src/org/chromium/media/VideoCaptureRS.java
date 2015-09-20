// Copyright 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package org.chromium.media;

import android.annotation.TargetApi;
import android.content.Context;
import android.graphics.ImageFormat;
import android.os.Build;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.util.Size;
import android.view.Surface;

import org.chromium.base.JNINamespace;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.intel.camera.toolkit.depth.Camera;
import com.intel.camera.toolkit.depth.Camera.Facing;
import com.intel.camera.toolkit.depth.Camera.Type;
import com.intel.camera.toolkit.depth.DepthUtils;
import com.intel.camera.toolkit.depth.Image;
import com.intel.camera.toolkit.depth.ImageSet;
import com.intel.camera.toolkit.depth.ImageInfo;
import com.intel.camera.toolkit.depth.OnSenseManagerHandler;
import com.intel.camera.toolkit.depth.RSPixelFormat;
import com.intel.camera.toolkit.depth.StreamProfile;
import com.intel.camera.toolkit.depth.StreamProfileSet;
import com.intel.camera.toolkit.depth.StreamType;
import com.intel.camera.toolkit.depth.StreamTypeSet;
import com.intel.camera.toolkit.depth.sensemanager.SenseManager;


/**
 * This class implements Video Capture using Camera2 API, introduced in Android
 * API 21 (L Release). Capture takes place in the current Looper, while pixel
 * download takes place in another thread used by ImageReader. A number of
 * static methods are provided to retrieve information on current system cameras
 * and their capabilities, using android.hardware.camera2.CameraManager.
 **/
@JNINamespace("media")
@TargetApi(Build.VERSION_CODES.LOLLIPOP)
public class VideoCaptureRS extends VideoCapture {

    private static final String TAG = "VideoCaptureRS";

    private byte[] mCapturedData;

    private static SenseManager mSenseManager = null;

    VideoCaptureRS(Context context,
                   int id,
                   long nativeVideoCaptureDeviceAndroid) {
        super(context, id, nativeVideoCaptureDeviceAndroid);
    }

    public static void setSenseManager(SenseManager sm) {
        mSenseManager = sm;
    }

    public static SenseManager getSenseManager() {
        return mSenseManager;
    }

    private StreamProfileSet getUserProfiles() {
        StreamProfileSet set = new StreamProfileSet();
        StreamProfile colorProfile = new StreamProfile(640, 480, RSPixelFormat.YV12, 30, StreamType.COLOR);
        StreamProfile depthProfile = new StreamProfile(480, 360, RSPixelFormat.Z16, 30, StreamType.DEPTH);
        set.set(StreamType.COLOR, colorProfile);
        set.set(StreamType.DEPTH, depthProfile);

        return set;
    }

    OnSenseManagerHandler mSenseEventHandler = new OnSenseManagerHandler()
    {
        @Override
        public void onSetProfile(Camera.CaptureInfo profiles) {
            Log.i(TAG, "OnSetProfile");
            // Configure Color Plane
            StreamProfile cs = profiles.getStreamProfiles().get(StreamType.COLOR);
            if(null == cs) {
                Log.e( TAG, "Error: NULL INDEX_COLOR");
            } else {
                Log.i( TAG, "Configuring color with format " + cs.Format + " for width " + cs.Width + " and height " + cs.Height);
            }

            // Configure Depth Plane
            StreamProfile ds = profiles.getStreamProfiles().get(StreamType.DEPTH);
            if(null == ds) {
                Log.e( TAG, "Error: NULL INDEX_DEPTH");
            } else {
                Log.i( TAG, "Configuring DisplayMode (DEPTH_RAW_GRAYSCALE): format " + ds.Format + " for width " + ds.Width + " and height " + ds.Height);
            }
            Log.i(TAG, "Camera Calibration: \n" + profiles.getCalibrationData());
        }
  
        
        @Override
        public void onNewSample(ImageSet images) {
            Image color = images.acquireImage(StreamType.COLOR);
            Image depth = images.acquireImage(StreamType.DEPTH);
        
            if (color == null) Log.i(TAG, "color is null");
            if (depth == null) Log.i(TAG, "depth is null");

            
            if (null != color) {
                byte[] imageBuffer = color.getImageBuffer().array();
                Log.i(TAG, "imageBuffer length " + imageBuffer.length + " " + "mCapturedData length " + mCapturedData.length);
                for (int i = 0; i < mCapturedData.length; ++i) {
                    mCapturedData[i] = imageBuffer[i];
                }
                nativeOnFrameAvailable(mNativeVideoCaptureDeviceAndroid,
                                       mCapturedData,
                                       mCapturedData.length,
                                       getCameraRotation());
            }
        }


        @Override
        public void onError(StreamProfileSet profile, int error) {
            Log.e(TAG, "Error: " + error);
        }
    };


    @Override
    public boolean allocate(int width, int height, int frameRate) {
        Log.i(TAG, "allocate: requested (" + width + "x" + height + ")@" + frameRate + "fps");

        // |mCaptureFormat| is also used to configure the ImageReader.
        mCaptureFormat = new CaptureFormat(width, height, frameRate, ImageFormat.YV12);
        int expectedFrameSize = mCaptureFormat.mWidth * mCaptureFormat.mHeight
                * ImageFormat.getBitsPerPixel(mCaptureFormat.mPixelFormat) / 8;
        mCapturedData = new byte[expectedFrameSize];
        return true;
    }

    @Override
    public boolean startCapture() {
        Log.i(TAG, "startCapture");
        try {
            getSenseManager().enableStreams(mSenseEventHandler, getUserProfiles(), null);
        } catch(Exception e) {
           Log.e(TAG, "Exception:" + e.getMessage());
           e.printStackTrace();
        }
        return true;
    }

    @Override
    public boolean stopCapture() {
        Log.i(TAG, "stopCapture");
        try {
            getSenseManager().close();
        } catch(Exception e) {
           Log.e(TAG, "Exception:" + e.getMessage());
           e.printStackTrace();
        }
        return true;
    }

    @Override
    public void deallocate() {
        Log.i(TAG, "deallocate");
    }
}
