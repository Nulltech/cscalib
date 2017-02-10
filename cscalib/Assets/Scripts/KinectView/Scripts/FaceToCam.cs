using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Windows.Kinect;
using Microsoft.Kinect.Face;
using System.IO;

public class FaceToCam : MonoBehaviour
{
    private KinectSensor kinectSensor;
    private int bodyCount;
    private Body[] bodies;
    private FaceFrameSource[] faceFrameSources;
    private FaceFrameReader[] faceFrameReaders;
    public GameObject bodyManager;

    //this is our game camera
    public GameObject face;

    StreamWriter sx;
    StreamWriter sy;
    StreamWriter kx;

    //my value 0.005
    public float qq;

    //my value 0.01
    public float rr;

    //my value 0.0001
    public float init_state;

    //bigger values faster rotations and more noise. my value 10
    public float headSmooth;

    private float last_x = 0;
    private float[] last;
    private float last_x3 = 0;

    private float last_y = 0;

    private float last_mod = 0;


    private int updateFrame;

    KalmanFilterSimple1D kalman_X;
    KalmanFilterSimple1D kalman_Y;
    KalmanFilterSimple1D kalman_mod;

    void Start()
    {
        updateFrame = 0;
        kalman_X = new KalmanFilterSimple1D(f: 1, h: 1, q: qq, r: rr);
        kalman_Y = new KalmanFilterSimple1D(f: 1, h: 1, q: qq, r: rr);
        kalman_mod = new KalmanFilterSimple1D(f: 1, h: 1, q: qq, r: rr);

        sx = new StreamWriter("coords_X.txt");
        kx = new StreamWriter("coords_KX.txt");


        // one sensor is currently supported
        kinectSensor = KinectSensor.GetDefault();


        // set the maximum number of bodies that would be tracked by Kinect
        bodyCount = kinectSensor.BodyFrameSource.BodyCount;

        // allocate storage to store body objects
        bodies = new Body[bodyCount];

        // specify the required face frame results
        FaceFrameFeatures faceFrameFeatures =
            FaceFrameFeatures.BoundingBoxInColorSpace
                | FaceFrameFeatures.PointsInColorSpace
                | FaceFrameFeatures.BoundingBoxInInfraredSpace
                | FaceFrameFeatures.PointsInInfraredSpace
                | FaceFrameFeatures.RotationOrientation
                | FaceFrameFeatures.FaceEngagement
                | FaceFrameFeatures.Glasses
                | FaceFrameFeatures.Happy
                | FaceFrameFeatures.LeftEyeClosed
                | FaceFrameFeatures.RightEyeClosed
                | FaceFrameFeatures.LookingAway
                | FaceFrameFeatures.MouthMoved
                | FaceFrameFeatures.MouthOpen;

        // create a face frame source + reader to track each face in the FOV
        faceFrameSources = new FaceFrameSource[bodyCount];
        faceFrameReaders = new FaceFrameReader[bodyCount];
        for (int i = 0; i < bodyCount; i++)
        {
            // create the face frame source with the required face frame features and an initial tracking Id of 0
            faceFrameSources[i] = FaceFrameSource.Create(kinectSensor, 0, faceFrameFeatures);

            // open the corresponding reader
            faceFrameReaders[i] = faceFrameSources[i].OpenReader();
        }
    }

    void LateUpdate()
    {
        if (updateFrame < 1)
        {
            updateFrame++;
            return;
        }
        updateFrame = 0;
        // get bodies either from BodySourceManager object get them from a BodyReader
        var bodySourceManager = bodyManager.GetComponent<BodySourceManager>();
        bodies = bodySourceManager.GetData();
        if (bodies == null)
        {
            return;
        }

        // iterate through each body and update face source
        for (int i = 0; i < bodyCount; i++)
        {
            // check if a valid face is tracked in this face source				
            if (faceFrameSources[i].IsTrackingIdValid)
            {
                using (FaceFrame frame = faceFrameReaders[i].AcquireLatestFrame())
                {
                    if (frame != null)  
                    {
                        if (frame.TrackingId == 0)
                        {
                            continue;
                        }

                        // do something with result
                        var result = frame.FaceFrameResult.FaceRotationQuaternion;

                        var facepoints = frame.FaceFrameResult.FacePointsInColorSpace;

                        if (last_x == 0)
                        {
                            last_x = face.transform.rotation.x;
                        }
                        if (last_y == 0)
                        {
                            last_y = face.transform.rotation.y;
                        }

                        sx.WriteLine((-result.X).ToString("0.000000"), true);
                        sx.Flush();

                        kalman_Y.SetState(last_y, init_state);
                        kalman_Y.Correct(-result.Y);

                        kalman_X.SetState(last_x, init_state);
                        kalman_X.Correct(-result.X);
                        float mod_x = last_x % -result.X;

                        kalman_mod.SetState(last_mod, init_state);
                        kalman_mod.Correct(mod_x);

                        //small shakes
                        if (mod_x > 0.01f && mod_x < 0.08f)
                        {

                            mod_x = mod_x / 2;
                        }


                        face.transform.localRotation = Quaternion.Lerp(face.transform.localRotation, new Quaternion((float)kalman_X.State, (float)kalman_Y.State, face.transform.localRotation.z, face.transform.localRotation.w), Time.deltaTime * headSmooth * mod_x + 0.3f);
                        kx.WriteLine(kalman_X.State.ToString("0.000000"), true);
                        kx.Flush();
                        last_y = (float)kalman_Y.State;
                        last_x = (float)kalman_X.State;
                        last_mod = (float)kalman_mod.State;


                        //updateFrame = !updateFrame;
                    }
                }
            }
            else
            {
                // check if the corresponding body is tracked 
                if (bodies[i].IsTracked)
                {
                    // update the face frame source to track this body
                    faceFrameSources[i].TrackingId = bodies[i].TrackingId;
                }
            }
        }

    }
}