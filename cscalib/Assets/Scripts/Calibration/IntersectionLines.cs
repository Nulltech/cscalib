using UnityEngine;
using System.Collections;
using Accord.MachineLearning;
using Accord.Math;
using Accord.Math.Decompositions;
using System.Collections.Generic;

public class LinesIntersectionResult
{
    public UnityEngine.Vector3 intersection;
    public int[] inliersIndexes;

    public LinesIntersectionResult(UnityEngine.Vector3 v3)
    {
        intersection = v3;
    }
}

public class LinesIntersection
{
    private static float evaluateDistance(TF line, UnityEngine.Vector3 intersectionPoint)
    {
        UnityEngine.Matrix4x4 m = Utils.getMatrix4x4FromTF(line);
        float a = m.m02;
        float b = m.m12;
        float c = m.m22;
        float x = intersectionPoint.x;
        float y = intersectionPoint.y;
        float z = intersectionPoint.z;
        float x0 = line.position.x;
        float y0 = line.position.y;
        float z0 = line.position.z;

        float t = (a * (x - x0) + b * (y - y0) + c * (z - z0)) / (Mathf.Pow(a, 2) + Mathf.Pow(b, 2) + Mathf.Pow(c, 2));
	
        float x1 = x0 + a * t;
        float y1 = y0 + b * t;
        float z1 = z0 + c * t;

		float distance = Mathf.Pow(x1 - x, 2) + Mathf.Pow(y1 - y, 2) + Mathf.Pow(z1 - z, 2);
        return distance;
    }

    public static float evaluateDistance(TF line, LinesIntersectionResult intersectionPoint)
    {
        return evaluateDistance(line, intersectionPoint.intersection);
    }

    public static float evaluateDistance(List<TF> lines, UnityEngine.Vector3 point)
    {
        float sumOfDist = 0;
        foreach(var line in lines)
        {
            sumOfDist += evaluateDistance(line, point);
        }
        return sumOfDist;
    }

    public static LinesIntersectionResult findIntersection(ref List<TF> lines)
    {
        double[,] A = new double[lines.Count * 3, 3];
        double[,] B = new double[lines.Count * 3, 1];
        for (int i = 0; i < lines.Count; ++i)
        {
            UnityEngine.Matrix4x4 m = Utils.getMatrix4x4FromTF(lines[i]);

            float a = m.m02;
            float b = m.m12;
            float c = m.m22;

            double[,] b_M = new double[3, 3];
            b_M[0, 0] = Mathf.Pow(a, 2) - 1;
            b_M[0, 1] = a * b;
            b_M[0, 2] = a * c;
            b_M[1, 0] = a * b;
            b_M[1, 1] = Mathf.Pow(b, 2) - 1;
            b_M[1, 2] = b * c;
            b_M[2, 0] = a * c;
            b_M[2, 1] = b * c;
            b_M[2, 2] = Mathf.Pow(c, 2) - 1;

            double[] xyz = new double[3];
            xyz[0] = lines[i].position.x;
            xyz[1] = lines[i].position.y;
            xyz[2] = lines[i].position.z;

            double[] B_tmp = Accord.Math.Matrix.Dot(b_M, xyz);
            B[(i * 3), 0] = B_tmp[0];
            B[(i * 3) + 1, 0] = B_tmp[1];
            B[(i * 3) + 2, 0] = B_tmp[2];

            double[,] A_tmp = new double[3, 3];
            A_tmp[0, 0] = Mathf.Pow(a, 2) - 1;
            A_tmp[0, 1] = a * b;
            A_tmp[0, 2] = a * c;
            A_tmp[1, 0] = a * b;
            A_tmp[1, 1] = Mathf.Pow(b, 2) - 1;
            A_tmp[1, 2] = b * c;
            A_tmp[2, 0] = a * c;
            A_tmp[2, 1] = b * c;
            A_tmp[2, 2] = Mathf.Pow(c, 2) - 1;

            A[(i * 3), 0] = A_tmp[0, 0];
            A[(i * 3), 1] = A_tmp[0, 1];
            A[(i * 3), 2] = A_tmp[0, 2];
            A[(i * 3) + 1, 0] = A_tmp[1, 0];
            A[(i * 3) + 1, 1] = A_tmp[1, 1];
            A[(i * 3) + 1, 2] = A_tmp[1, 2];
            A[(i * 3) + 2, 0] = A_tmp[2, 0];
            A[(i * 3) + 2, 1] = A_tmp[2, 1];
            A[(i * 3) + 2, 2] = A_tmp[2, 2];
        }

        UnityEngine.Vector3 intersection = new UnityEngine.Vector3();

        QrDecomposition qr = new QrDecomposition(A);
        double[,] result = qr.Solve(B);

        intersection.x = (float)result[0, 0];
        intersection.y = (float)result[1, 0];
        intersection.z = (float)result[2, 0];

        return new LinesIntersectionResult(intersection);
    }
}

public static class Utils
{
    public static UnityEngine.Matrix4x4 getMatrixFromQuaternion(Quaternion q)
    {
        UnityEngine.Matrix4x4 matrix = new UnityEngine.Matrix4x4();
        float sqw = q.w * q.w;
        float sqx = q.x * q.x;
        float sqy = q.y * q.y;
        float sqz = q.z * q.z;

        // invs (inverse square length) is only required if quaternion is not already normalised
        float invs = 1 / (sqx + sqy + sqz + sqw);
        matrix.m00 = (sqx - sqy - sqz + sqw) * invs; // since sqw + sqx + sqy + sqz = 1/invs*invs
        matrix.m11 = (-sqx + sqy - sqz + sqw) * invs;
        matrix.m22 = (-sqx - sqy + sqz + sqw) * invs;

        float tmp1 = q.x * q.y;
        float tmp2 = q.z * q.w;
        matrix.m10 = 2.0f * (tmp1 + tmp2) * invs;
        matrix.m01 = 2.0f * (tmp1 - tmp2) * invs;

        tmp1 = q.x * q.z;
        tmp2 = q.y * q.w;
        matrix.m20 = 2.0f * (tmp1 - tmp2) * invs;
        matrix.m02 = 2.0f * (tmp1 + tmp2) * invs;
        tmp1 = q.y * q.z;
        tmp2 = q.x * q.w;
        matrix.m21 = 2.0f * (tmp1 + tmp2) * invs;
        matrix.m12 = 2.0f * (tmp1 - tmp2) * invs;

        return matrix;
    }

    public static UnityEngine.Matrix4x4 getMatrix4x4FromTF(TF tf)
    {
        UnityEngine.Vector3 dummyScale = new UnityEngine.Vector3(1, 1, 1);
        return UnityEngine.Matrix4x4.TRS(tf.position, tf.rotation, dummyScale);
    }

    public static LinesIntersectionResult findIntersectionWithRansac(List<TF> lines, double threshold, double ransacProbability = 0.95f,  int max_evaluations = 1000)
    {
        RANSAC<LinesIntersectionResult> ransac = new RANSAC<LinesIntersectionResult>(3);
        ransac.Probability = ransacProbability;
        ransac.Threshold = threshold;
        ransac.MaxEvaluations = max_evaluations;

        ransac.Fitting = delegate (int[] sample)
        {
            List<TF> inputs = lines.Get(sample);
            return LinesIntersection.findIntersection(ref inputs);
        };

        ransac.Degenerate = delegate (int[] sample)
        {
            return false;
        };

        ransac.Distances = delegate (LinesIntersectionResult li, double th)
        {
            List<int> inliers = new List<int>();
            float distance;
            for (int i = 0; i < lines.Count; ++i)
            {
                distance = Mathf.Sqrt(LinesIntersection.evaluateDistance(lines[i], li));
                if (distance < th)
                {
                    inliers.Add(i);
                }
            }
            return inliers.ToArray();
        };

        int[] inlierIndices;
        LinesIntersectionResult result = ransac.Compute(lines.Count, out inlierIndices);
        result.inliersIndexes = inlierIndices;

        if (result == null)
        {
            Debug.LogError("The RANSAC algorithm did not find any inliers and no model was created");
            return null;
        }

        List<TF> inlierLines = new List<TF>();
        foreach (var inlierIdx in inlierIndices)
        {
            inlierLines.Add(lines[inlierIdx]);
        }

        return result;
    }

    public static LMResult LM(TF guess, ErrorFunctionArgs args)
    {
        UnityEngine.Vector3 rpy = guess.rotation.eulerAngles;
        double[] x = new double[] { guess.position.x, guess.position.y, guess.position.z, rpy[0], rpy[1], rpy[2], 0.03, 0.03};
        double epsg = 0.0000000001;
        double epsf = 0;
        double epsx = 0;
        int maxits = 0;
        alglib.minlmstate state;
        alglib.minlmreport rep;

        alglib.minlmcreatev(4, x, 0.001, out state);
        alglib.minlmsetcond(state, epsg, epsf, epsx, maxits);
        alglib.minlmoptimize(state, errorFunction, null, args);
        alglib.minlmresults(state, out x, out rep);

        Debug.Log("LM report result termination type: " + rep.terminationtype);
        Debug.Log("LM report result num iteration: " + rep.iterationscount);

        UnityEngine.Vector3 result_point = new UnityEngine.Vector3((float)x[0], (float)x[1], (float)x[2]);
        UnityEngine.Quaternion result_rotation = UnityEngine.Quaternion.Euler((float)x[3], (float)x[4] , (float)x[5]);

        Debug.Log("value m width : " + (float)x[6]);
        Debug.Log("value m height : " + (float)x[7]);


        UnityEngine.Vector3 ur = result_point + result_rotation * new UnityEngine.Vector3(args.monitorWidth, 0, 0);
        UnityEngine.Vector3 bl = result_point + result_rotation * new UnityEngine.Vector3(0, -args.monitorHeight, 0);
        UnityEngine.Vector3 br = result_point + result_rotation * new UnityEngine.Vector3(args.monitorWidth, -args.monitorHeight, 0);

        List<UnityEngine.Vector3> corners = new List<UnityEngine.Vector3>();
        corners.Add(result_point); corners.Add(ur); corners.Add(bl); corners.Add(br);

        return new LMResult(corners, result_rotation);
    }

    private static void errorFunction(double[] x, double[] fi, object obj)
    {
        ErrorFunctionArgs args = (ErrorFunctionArgs)obj;

        UnityEngine.Vector3 ul = new UnityEngine.Vector3((float)x[0], (float)x[1], (float)x[2]);
        UnityEngine.Quaternion R = UnityEngine.Quaternion.Euler((float)x[3], (float)x[4], (float)x[5]);

        UnityEngine.Vector3 ur = ul + R * new UnityEngine.Vector3(args.monitorWidth + (float)x[6], 0, 0);
        UnityEngine.Vector3 bl = ul + R * new UnityEngine.Vector3(0, -args.monitorHeight + (float)x[7], 0);
        UnityEngine.Vector3 br = ul + R * new UnityEngine.Vector3(args.monitorWidth + (float)x[6], -args.monitorHeight + (float)x[7], 0);

        fi[0] = LinesIntersection.evaluateDistance(args.samples[0], ul);
        fi[1] = LinesIntersection.evaluateDistance(args.samples[1], ur);
        fi[2] = LinesIntersection.evaluateDistance(args.samples[2], bl);
        fi[3] = LinesIntersection.evaluateDistance(args.samples[3], br);

        return;
    }
}

public struct ErrorFunctionArgs
{
    public Dictionary<int, List<TF>> samples;
    public List<UnityEngine.Vector3> cornerRansac;
    public float monitorWidth;
    public float monitorHeight;

    public ErrorFunctionArgs(Dictionary<int, List<TF>> _s, List<UnityEngine.Vector3> _cr, float _mw, float _mh)
    {
        samples = _s;
        cornerRansac = _cr;
        monitorWidth = _mw;
        monitorHeight = _mh;
    }
}

public struct LMResult
{
    public List<UnityEngine.Vector3> cornersPosition;
    public UnityEngine.Quaternion rotation;

    public LMResult(List<UnityEngine.Vector3> _cp, Quaternion _rot)
    {
        cornersPosition = _cp;
        rotation = _rot;
    }
}