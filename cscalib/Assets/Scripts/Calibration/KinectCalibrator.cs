using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices;
using UnityEngine.SceneManagement;
using Windows.Kinect;
using Microsoft.Kinect.Face;
using System.Collections.Generic;
using UnityEngine.UI;

public class KinectCalibrator : MonoBehaviour
{
    //Kinect sensor
    private KinectSensor kinectSensor;
    private int bodyCount;
    private Body[] bodies;
    private FaceFrameSource[] faceFrameSources;
    private FaceFrameReader[] faceFrameReaders;

    public Text DbgDistance;

    public GameObject bodyManager;

    public GameObject cube;
    public GameObject[] joints = new GameObject[26];
    //public GameObject handHeadPointer;
    public GameObject handEyePointer;
    public GameObject menu;
    public float monitorHeight;
    public float monitorWidth;

    /** LPF **/
    float[] smoothedValM2 = new float[4];
    float[] smoothedValM3 = new float[4];
    float filterValM2, filterValM3;


    /** LM **/
    private TF handHeadWrtKinectTf;
    private TF handEyeWrtKinectTf;

    /** Lines per corner **/
    int counter = 0;
    int currentCorner = 0;
    Dictionary<int, List<TF>> handEyeLines = new Dictionary<int, List<TF>>();
    Dictionary<int, List<TF>> handHeadLines = new Dictionary<int, List<TF>>();
    Dictionary<int, List<GameObject>> handPointers = new Dictionary<int, List<GameObject>>();

    public int Lines_per_corner = 10;
    public float Ransac_Threshold = 0.01f;
    public double Ransac_Probability = 0.95;

    public Vector3 UpperLeftGuessWrtKinect;

    public Material inlierMat;

    enum MonitorCorner { UPPER_LEFT_CORNER = 0, UPPER_RIGHT_CORNER = 1, BOTTOM_LEFT_CORNER = 2, BOTTOM_RIGHT_CORNER = 3, CENTER = 4 };

    void Start()
    {
        for (int i = 0; i < 4; ++i)
        {
            handEyeLines.Add(i, new List<TF>());
            handHeadLines.Add(i, new List<TF>());
            handPointers.Add(i, new List<GameObject>());
        }

        /** LPF init **/
        for (int i = 0; i < 4; ++i)
        {
            smoothedValM2[i] = smoothedValM2[i] = 0.0f;
        }
        filterValM2 = filterValM3 = 0.8f;

        /** KINECT **/
        kinectSensor = KinectSensor.GetDefault();
        // set the maximum number of bodies that would be tracked by Kinect
        bodyCount = kinectSensor.BodyFrameSource.BodyCount;
        //bodyCount = 1;

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

        /** END-KINECT **/

        for (int i = 0; i < 26; ++i)
        {
            joints[i] = GameObject.Instantiate(cube);
        }


        menu.SetActive(true);

    }

    public void doCalibration()
    {
        menu.SetActive(false);
    }

    // Update is called once per frame
    void Update()
    {
        // get bodies either from BodySourceManager object get them from a BodyReader
        var bodySourceManager = bodyManager.GetComponent<BodySourceManager>();
        bodies = bodySourceManager.GetData();

        if (bodies == null)
        {
            return;
        }

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

                        //body joints update
                        //update name & position
                        foreach (var joint in bodies[i].Joints)
                        {
                            joints[(int)joint.Key].transform.name = joint.Key.ToString();
                            joints[(int)joint.Key].transform.position = new Vector3(-joint.Value.Position.X, joint.Value.Position.Y, joint.Value.Position.Z);
                        }
                        //update orientation
                        foreach (var joint in bodies[i].JointOrientations)
                        {
                            joints[(int)joint.Key].transform.rotation = new Quaternion(joint.Value.Orientation.X, -joint.Value.Orientation.Y, -joint.Value.Orientation.Z, joint.Value.Orientation.W);
                        }

                        // do something with result
                        var result = frame.FaceFrameResult.FaceRotationQuaternion;

                        var eyePosition = frame.FaceFrameResult.FacePointsInColorSpace[FacePointType.EyeRight];

                        ColorFrame colorFrame = frame.ColorFrameReference.AcquireFrame();
                        if (colorFrame != null)
                        {
                            FrameDescription colorFrameDescription = colorFrame.FrameDescription;
                            DepthFrame depthFrame = frame.DepthFrameReference.AcquireFrame();
                            if (depthFrame != null)
                            {
                                FrameDescription depthFrameDescription = depthFrame.FrameDescription;

                                CameraSpacePoint[] cameraPoints = new CameraSpacePoint[colorFrameDescription.Width * colorFrameDescription.Height];
                                ushort[] rawDepthPixelData = new ushort[depthFrameDescription.Width * depthFrameDescription.Height];
                                depthFrame.CopyFrameDataToArray(rawDepthPixelData);
                                kinectSensor.CoordinateMapper.MapColorFrameToCameraSpace(rawDepthPixelData, cameraPoints);
                                var eyeWorldCoordinate = cameraPoints[(int)eyePosition.Y * colorFrameDescription.Width + (int)eyePosition.X];

                                //Debug.Log(new Vector3(-eyeWorldCoordinate.X, eyeWorldCoordinate.Y, eyeWorldCoordinate.Z));

                                joints[25].transform.name = FacePointType.EyeRight.ToString();
                                joints[25].transform.position = new Vector3(-eyeWorldCoordinate.X, eyeWorldCoordinate.Y, eyeWorldCoordinate.Z);
                                joints[25].GetComponent<Renderer>().material.color = UnityEngine.Color.red;

                                DbgDistance.text = "" + eyeWorldCoordinate.Z;

                                Vector3 hand = joints[(int)JointType.HandTipLeft].transform.position;
                                Vector3 head = joints[(int)JointType.Head].transform.position;   //HEAD
                                Vector3 eye = joints[25].transform.position; //EYE_RIGHT

                                TF handHeadUnityTf = computeHandPointerTransform(hand, head, true, 0, true);
                                //handHeadUnityTf.setToGameObject(ref handHeadPointer);

                                TF handEyeUnityTf = computeHandPointerTransform(hand, eye, true, 1, true);
                                handEyeUnityTf.setToGameObject(ref handEyePointer);

                                /** handPointer w.r.t. kinect (right-handed) **/
                                Vector3 handWrtKinect = new Vector3(-hand.x, hand.y, hand.z);
                                Vector3 headWrtKinect = new Vector3(-head.x, head.y, head.z);
                                Vector3 eyeWrtKinect = new Vector3(-eye.x, eye.y, eye.z);

                                handHeadWrtKinectTf = computeHandPointerTransform(handWrtKinect, headWrtKinect, true, 2, false);
                                handEyeWrtKinectTf = computeHandPointerTransform(handWrtKinect, eyeWrtKinect, true, 3, false);

                                /** End Hand Pointer **/

                                depthFrame.Dispose();
                                depthFrame = null;
                            }
                            colorFrame.Dispose();
                            colorFrame = null;
                        }
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

        if (Input.GetKeyUp(KeyCode.KeypadEnter))
        {
            DoSaveCalibrationLine();
            print("save calibration line");
        }
    }

    private TF computeHandPointerTransform(Vector3 hand, Vector3 head, bool applyFilter, int LPF_index, bool leftHanded = true)
    {
        TF tf;
        Vector3 vHeadHand = new Vector3(hand.x - head.x, hand.y - head.y, hand.z - head.z);
        vHeadHand.Normalize();

        float m2 = vHeadHand[0] / vHeadHand[2];
        float m3 = -vHeadHand[1] / vHeadHand[2];

        if (applyFilter)
        {
            applyLPF(ref m2, ref m3, LPF_index);
        }

        tf.position = hand;

        if (leftHanded)
        {
            tf.rotation = Quaternion.Euler(Mathf.Rad2Deg * Mathf.Atan(m3) + Mathf.Rad2Deg * Mathf.PI / 2, Mathf.Rad2Deg * Mathf.Atan(m2), 0);
        }
        else
        {
            //Quaternion r = Quaternion.Euler(0, Mathf.Rad2Deg * Mathf.Atan(m2), 0);
            //Quaternion r2 = Quaternion.Euler(Mathf.Rad2Deg * Mathf.Atan(m3), 0, 0);
            //r = r * r2;
            //tf.rotation = r2;
            tf.rotation = Quaternion.Euler(Mathf.Rad2Deg * Mathf.Atan(m3), Mathf.Rad2Deg * Mathf.Atan(m2), 0);
        }

        return tf;
    }

    void applyLPF(ref float m2, ref float m3, int idx)
    {
        smoothedValM2[idx] = (m2 * (1 - filterValM2)) + (smoothedValM2[idx] * filterValM2);
        smoothedValM3[idx] = (m3 * (1 - filterValM3)) + (smoothedValM3[idx] * filterValM3);

        m2 = smoothedValM2[idx];
        m3 = smoothedValM3[idx];
    }

    void DoSaveCalibrationLine()
    {
        if (currentCorner == 4)
        {
            /** Find intersection with least mean squared **/
            //FindIntersection();
            LinesIntersectionResult result_no_ransac, result_ransac;
            Vector3 guessUL = new Vector3(0, 0, 0);
            List<Vector3> cornerNoRansacWrtKinect = new List<Vector3>();
            List<Vector3> cornerRansacWrtKinect = new List<Vector3>();
            for (int i = 0; i < 4; ++i)
            {
                MonitorCorner mc_ref = (MonitorCorner)i;
                List<TF> dummy = handEyeLines[i];
                result_no_ransac = LinesIntersection.findIntersection(ref dummy);
                IniFile.IniWriteValue("CORNERS_POSITION_WITHOUT_RANSAC", mc_ref.ToString() + "_X", result_no_ransac.intersection.x.ToString());
                IniFile.IniWriteValue("CORNERS_POSITION_WITHOUT_RANSAC", mc_ref.ToString() + "_Y", result_no_ransac.intersection.y.ToString());
                IniFile.IniWriteValue("CORNERS_POSITION_WITHOUT_RANSAC", mc_ref.ToString() + "_Z", result_no_ransac.intersection.z.ToString());

                cornerNoRansacWrtKinect.Add(new Vector3(result_no_ransac.intersection.x, result_no_ransac.intersection.y, result_no_ransac.intersection.z));

                //save UPPER_LEFT_CORNER because it's our favourite guess
                if ((MonitorCorner)i == MonitorCorner.UPPER_LEFT_CORNER)
                {
                    guessUL = result_no_ransac.intersection;
                }

                //draw cube 
                drawCornerCube(mc_ref, result_no_ransac.intersection, "CORNERS_POSITION_WITHOUT_RANSAC", UnityEngine.Color.red);
            }
            ErrorFunctionArgs data = new ErrorFunctionArgs(handEyeLines, cornerNoRansacWrtKinect, monitorWidth, monitorHeight);

            guessUL = UpperLeftGuessWrtKinect;
            TF guess = new TF(Quaternion.Euler(-0.1f, -0.1f, -0.1f), guessUL);
            LMResult resultLM = Utils.LM(guess, data);

            /** INI FILE **/
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", MonitorCorner.UPPER_LEFT_CORNER.ToString() + "_X", resultLM.cornersPosition[0][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", MonitorCorner.UPPER_LEFT_CORNER.ToString() + "_Y", resultLM.cornersPosition[0][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", MonitorCorner.UPPER_LEFT_CORNER.ToString() + "_Z", resultLM.cornersPosition[0][2].ToString());
            drawCornerCube((MonitorCorner)0, resultLM.cornersPosition[0], "OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", UnityEngine.Color.yellow);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", MonitorCorner.UPPER_RIGHT_CORNER.ToString() + "_X", resultLM.cornersPosition[1][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", MonitorCorner.UPPER_RIGHT_CORNER.ToString() + "_Y", resultLM.cornersPosition[1][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", MonitorCorner.UPPER_RIGHT_CORNER.ToString() + "_Z", resultLM.cornersPosition[1][2].ToString());
            drawCornerCube((MonitorCorner)1, resultLM.cornersPosition[1], "OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", UnityEngine.Color.yellow);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", MonitorCorner.BOTTOM_LEFT_CORNER.ToString() + "_X", resultLM.cornersPosition[2][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", MonitorCorner.BOTTOM_LEFT_CORNER.ToString() + "_Y", resultLM.cornersPosition[2][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", MonitorCorner.BOTTOM_LEFT_CORNER.ToString() + "_Z", resultLM.cornersPosition[2][2].ToString());
            drawCornerCube((MonitorCorner)2, resultLM.cornersPosition[2], "OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", UnityEngine.Color.yellow);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", MonitorCorner.BOTTOM_RIGHT_CORNER.ToString() + "_X", resultLM.cornersPosition[3][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", MonitorCorner.BOTTOM_RIGHT_CORNER.ToString() + "_Y", resultLM.cornersPosition[3][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", MonitorCorner.BOTTOM_RIGHT_CORNER.ToString() + "_Z", resultLM.cornersPosition[3][2].ToString());
            drawCornerCube((MonitorCorner)3, resultLM.cornersPosition[3], "OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", UnityEngine.Color.yellow);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", "ANGLE_DEG_X", resultLM.rotation.eulerAngles[0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", "ANGLE_DEG_Y", resultLM.rotation.eulerAngles[1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)", "ANGLE_DEG_Z", resultLM.rotation.eulerAngles[2].ToString());

            Plane plane = new Plane();
            plane.Set3Points(cornerNoRansacWrtKinect[0], cornerNoRansacWrtKinect[1], cornerNoRansacWrtKinect[3]);
            float pitch = Mathf.Rad2Deg * (Mathf.Acos(Vector3.Dot(Vector3.right, plane.normal)) - (Mathf.PI / 2));
            float roll = Mathf.Rad2Deg * (Mathf.Acos(Vector3.Dot(Vector3.up, plane.normal)) - (Mathf.PI / 2));
            //float yaw = Vector3.Angle(Vector3.forward, plane.normal);
            float yaw = Mathf.Atan2((cornerNoRansacWrtKinect[1].y - cornerNoRansacWrtKinect[0].y), (cornerNoRansacWrtKinect[1].x - (cornerNoRansacWrtKinect[0].x))) * Mathf.Rad2Deg;
            //DO LM without outliers detection
            plane = new Plane();
            plane.Set3Points(cornerNoRansacWrtKinect[0], cornerNoRansacWrtKinect[1], cornerNoRansacWrtKinect[3]);
            pitch = Mathf.Rad2Deg * (Mathf.Acos(Vector3.Dot(Vector3.right, plane.normal)) - (Mathf.PI / 2));
            roll = Mathf.Rad2Deg * (Mathf.Acos(Vector3.Dot(Vector3.up, plane.normal)) - (Mathf.PI / 2));
            yaw = Mathf.Atan2((cornerNoRansacWrtKinect[1].y - cornerNoRansacWrtKinect[0].y), (cornerNoRansacWrtKinect[1].x - (cornerNoRansacWrtKinect[0].x))) * Mathf.Rad2Deg;
            guess = new TF(Quaternion.Euler(roll, pitch, yaw), cornerNoRansacWrtKinect[0]);
            resultLM = Utils.LM(guess, data);

            /** INI FILE **/
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", MonitorCorner.UPPER_LEFT_CORNER.ToString() + "_X", resultLM.cornersPosition[0][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", MonitorCorner.UPPER_LEFT_CORNER.ToString() + "_Y", resultLM.cornersPosition[0][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", MonitorCorner.UPPER_LEFT_CORNER.ToString() + "_Z", resultLM.cornersPosition[0][2].ToString());
            drawCornerCube((MonitorCorner)0, resultLM.cornersPosition[0], "OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", UnityEngine.Color.yellow);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", MonitorCorner.UPPER_RIGHT_CORNER.ToString() + "_X", resultLM.cornersPosition[1][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", MonitorCorner.UPPER_RIGHT_CORNER.ToString() + "_Y", resultLM.cornersPosition[1][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", MonitorCorner.UPPER_RIGHT_CORNER.ToString() + "_Z", resultLM.cornersPosition[1][2].ToString());
            drawCornerCube((MonitorCorner)1, resultLM.cornersPosition[1], "OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", UnityEngine.Color.yellow);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", MonitorCorner.BOTTOM_LEFT_CORNER.ToString() + "_X", resultLM.cornersPosition[2][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", MonitorCorner.BOTTOM_LEFT_CORNER.ToString() + "_Y", resultLM.cornersPosition[2][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", MonitorCorner.BOTTOM_LEFT_CORNER.ToString() + "_Z", resultLM.cornersPosition[2][2].ToString());
            drawCornerCube((MonitorCorner)2, resultLM.cornersPosition[2], "OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", UnityEngine.Color.yellow);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", MonitorCorner.BOTTOM_RIGHT_CORNER.ToString() + "_X", resultLM.cornersPosition[3][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", MonitorCorner.BOTTOM_RIGHT_CORNER.ToString() + "_Y", resultLM.cornersPosition[3][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", MonitorCorner.BOTTOM_RIGHT_CORNER.ToString() + "_Z", resultLM.cornersPosition[3][2].ToString());
            drawCornerCube((MonitorCorner)3, resultLM.cornersPosition[3], "OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", UnityEngine.Color.yellow);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", "ANGLE_DEG_X", resultLM.rotation.eulerAngles[0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", "ANGLE_DEG_Y", resultLM.rotation.eulerAngles[1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)", "ANGLE_DEG_Z", resultLM.rotation.eulerAngles[2].ToString());

            counter = 0;
            Dictionary<int, List<TF>> inliersLines = new Dictionary<int, List<TF>>();
            guessUL = new Vector3(0, 0, 0);
            for (int i = 0; i < 4; ++i)
            {
                inliersLines.Add(i, new List<TF>());
            }

            for (int i = 0; i < 4; ++i)
            {
                MonitorCorner mc_ref = (MonitorCorner)i;
                List<TF> dummy = handEyeLines[i];
                result_ransac = Utils.findIntersectionWithRansac(dummy, Ransac_Threshold, Ransac_Probability);
                IniFile.IniWriteValue("CORNERS_POSITION_RANSAC", mc_ref.ToString() + "_X", result_ransac.intersection.x.ToString());
                IniFile.IniWriteValue("CORNERS_POSITION_RANSAC", mc_ref.ToString() + "_Y", result_ransac.intersection.y.ToString());
                IniFile.IniWriteValue("CORNERS_POSITION_RANSAC", mc_ref.ToString() + "_Z", result_ransac.intersection.z.ToString());
                drawCornerCube(mc_ref, result_ransac.intersection, "CORNERS_POSITION_RANSAC", UnityEngine.Color.blue);

                cornerRansacWrtKinect.Add(new Vector3(result_ransac.intersection.x, result_ransac.intersection.y, result_ransac.intersection.z));

                if (i == 0)
                {
                    guessUL = result_ransac.intersection;
                }
                makeInlierVisible(i, result_ransac.inliersIndexes);
                foreach (var inlierIndex in result_ransac.inliersIndexes)
                {
                    inliersLines[i].Add(handEyeLines[i][inlierIndex]);
                }
            }

            data = new ErrorFunctionArgs(inliersLines, cornerRansacWrtKinect, monitorWidth, monitorHeight);
            guessUL = UpperLeftGuessWrtKinect;
            guess = new TF(Quaternion.Euler(-0.1f, -0.1f, -0.1f), guessUL);
            resultLM = Utils.LM(guess, data);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", MonitorCorner.UPPER_LEFT_CORNER.ToString() + "_X", resultLM.cornersPosition[0][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", MonitorCorner.UPPER_LEFT_CORNER.ToString() + "_Y", resultLM.cornersPosition[0][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", MonitorCorner.UPPER_LEFT_CORNER.ToString() + "_Z", resultLM.cornersPosition[0][2].ToString());
            drawCornerCube((MonitorCorner)0, resultLM.cornersPosition[0], "OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", UnityEngine.Color.green);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", MonitorCorner.UPPER_RIGHT_CORNER.ToString() + "_X", resultLM.cornersPosition[1][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", MonitorCorner.UPPER_RIGHT_CORNER.ToString() + "_Y", resultLM.cornersPosition[1][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", MonitorCorner.UPPER_RIGHT_CORNER.ToString() + "_Z", resultLM.cornersPosition[1][2].ToString());
            drawCornerCube((MonitorCorner)1, resultLM.cornersPosition[1], "OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", UnityEngine.Color.green);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", MonitorCorner.BOTTOM_LEFT_CORNER.ToString() + "_X", resultLM.cornersPosition[2][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", MonitorCorner.BOTTOM_LEFT_CORNER.ToString() + "_Y", resultLM.cornersPosition[2][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", MonitorCorner.BOTTOM_LEFT_CORNER.ToString() + "_Z", resultLM.cornersPosition[2][2].ToString());
            drawCornerCube((MonitorCorner)2, resultLM.cornersPosition[2], "OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", UnityEngine.Color.green);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", MonitorCorner.BOTTOM_RIGHT_CORNER.ToString() + "_X", resultLM.cornersPosition[3][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", MonitorCorner.BOTTOM_RIGHT_CORNER.ToString() + "_Y", resultLM.cornersPosition[3][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", MonitorCorner.BOTTOM_RIGHT_CORNER.ToString() + "_Z", resultLM.cornersPosition[3][2].ToString());
            drawCornerCube((MonitorCorner)3, resultLM.cornersPosition[3], "OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", UnityEngine.Color.green);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", "ANGLE_DEG_X", resultLM.rotation.eulerAngles[0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", "ANGLE_DEG_Y", resultLM.rotation.eulerAngles[1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)", "ANGLE_DEG_Z", resultLM.rotation.eulerAngles[2].ToString());

            plane = new Plane();
            plane.Set3Points(cornerRansacWrtKinect[0], cornerRansacWrtKinect[1], cornerRansacWrtKinect[3]);
            pitch = Mathf.Rad2Deg * (Mathf.Acos(Vector3.Dot(Vector3.right, plane.normal)) - (Mathf.PI / 2));
            roll = Mathf.Rad2Deg * (Mathf.Acos(Vector3.Dot(Vector3.up, plane.normal)) - (Mathf.PI / 2));
            yaw = Mathf.Atan2((cornerRansacWrtKinect[1].y - cornerRansacWrtKinect[0].y), (cornerRansacWrtKinect[1].x - (cornerRansacWrtKinect[0].x))) * Mathf.Rad2Deg;
            guess = new TF(Quaternion.Euler(roll, pitch, yaw), cornerRansacWrtKinect[0]);
            resultLM = Utils.LM(guess, data);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", MonitorCorner.UPPER_LEFT_CORNER.ToString() + "_X", resultLM.cornersPosition[0][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", MonitorCorner.UPPER_LEFT_CORNER.ToString() + "_Y", resultLM.cornersPosition[0][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", MonitorCorner.UPPER_LEFT_CORNER.ToString() + "_Z", resultLM.cornersPosition[0][2].ToString());
            drawCornerCube((MonitorCorner)0, resultLM.cornersPosition[0], "OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", UnityEngine.Color.green);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", MonitorCorner.UPPER_RIGHT_CORNER.ToString() + "_X", resultLM.cornersPosition[1][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", MonitorCorner.UPPER_RIGHT_CORNER.ToString() + "_Y", resultLM.cornersPosition[1][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", MonitorCorner.UPPER_RIGHT_CORNER.ToString() + "_Z", resultLM.cornersPosition[1][2].ToString());
            drawCornerCube((MonitorCorner)1, resultLM.cornersPosition[1], "OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", UnityEngine.Color.green);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", MonitorCorner.BOTTOM_LEFT_CORNER.ToString() + "_X", resultLM.cornersPosition[2][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", MonitorCorner.BOTTOM_LEFT_CORNER.ToString() + "_Y", resultLM.cornersPosition[2][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", MonitorCorner.BOTTOM_LEFT_CORNER.ToString() + "_Z", resultLM.cornersPosition[2][2].ToString());
            drawCornerCube((MonitorCorner)2, resultLM.cornersPosition[2], "OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", UnityEngine.Color.green);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", MonitorCorner.BOTTOM_RIGHT_CORNER.ToString() + "_X", resultLM.cornersPosition[3][0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", MonitorCorner.BOTTOM_RIGHT_CORNER.ToString() + "_Y", resultLM.cornersPosition[3][1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", MonitorCorner.BOTTOM_RIGHT_CORNER.ToString() + "_Z", resultLM.cornersPosition[3][2].ToString());
            drawCornerCube((MonitorCorner)3, resultLM.cornersPosition[3], "OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", UnityEngine.Color.green);

            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", "ANGLE_DEG_X", resultLM.rotation.eulerAngles[0].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", "ANGLE_DEG_Y", resultLM.rotation.eulerAngles[1].ToString());
            IniFile.IniWriteValue("OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)", "ANGLE_DEG_Z", resultLM.rotation.eulerAngles[2].ToString());

            menu.SetActive(true);

            return;
        }

        if (counter < Lines_per_corner)
        {
            //draw line on unity
            GameObject go = GameObject.Instantiate(handEyePointer);
            handPointers[currentCorner].Add(go);
            handHeadLines[currentCorner].Add(handHeadWrtKinectTf);
            handEyeLines[currentCorner].Add(handEyeWrtKinectTf);

            counter++;
        }
        else
        {
            currentCorner++;
            counter = 0;
            Debug.Log("New Corner -> " + ((MonitorCorner)currentCorner).ToString());
        }
    }

    private void makeInlierVisible(int monitorCorner, int[] inliers)
    {
        foreach (var inlier in inliers)
        {
            handPointers[monitorCorner][inlier].GetComponentInChildren<Renderer>().material = inlierMat;
        }
    }

    private void drawCornerCube(MonitorCorner mc, Vector3 pos, string name, UnityEngine.Color color)
    {
        GameObject go = Instantiate(cube);
        go.name = mc.ToString() + "_" + name;
        go.transform.position = new Vector3(-pos.x, pos.y, pos.z);     //to left handed (-x)
        go.GetComponent<Renderer>().material.color = color;
    }
}
