using UnityEngine;

namespace DeepseaAUV.AI
{
    /// <summary>
    /// 三次均匀 B 样条曲线生成器 (Cubic Uniform B-spline)
    /// 对 RRT* 给出的分段线性路径进行平滑，
    /// 输出 C2 连续的曲线及一阶/二阶导数（供曲率计算）。
    /// </summary>
    public static class BSplineSmoother
    {
        private static readonly float[] _coeffs = new float[]
        {
            1f/6f, 4f/6f, 1f/6f, 0f,       // 0 阶 (位置)
            -0.5f,   0f,    0.5f,    0f,     // 1 阶 (速度/切向)
            1f,     -2f,    1f,      0f,     // 2 阶 (加速度/曲率)
        };

        /// <summary>
        /// 用 B 样条平滑离散路径点。
        /// </summary>
        /// <param name="controlPoints">原始控制点（RRT* 路径）</param>
        /// <param name="samplesPerSegment">每段采样数</param>
        /// <param name="loop">是否闭合</param>
        /// <returns>平滑后的点序列</returns>
        public static Vector3[] Smooth(Vector3[] controlPoints, int samplesPerSegment = 8, bool loop = false)
        {
            if (controlPoints == null || controlPoints.Length < 3)
                return controlPoints;

            // Pad start/end to ensure curve passes through first and last points
            Vector3[] cp = PadControlPoints(controlPoints, loop);
            int nSeg = cp.Length - 3;
            int total = nSeg * samplesPerSegment + 1;
            var result = new Vector3[total];

            int idx = 0;
            for (int s = 0; s < nSeg; s++)
            {
                for (int i = 0; i < samplesPerSegment; i++)
                {
                    float t = i / (float)samplesPerSegment;
                    result[idx++] = EvaluatePosition(cp, s, t);
                }
            }
            result[total - 1] = cp[cp.Length - 2];
            return result;
        }

        public static Vector3[] GetTangents(Vector3[] controlPoints, int samplesPerSegment = 8, bool loop = false)
        {
            Vector3[] cp = PadControlPoints(controlPoints, loop);
            int nSeg = cp.Length - 3;
            int total = nSeg * samplesPerSegment + 1;
            var result = new Vector3[total];

            int idx = 0;
            for (int s = 0; s < nSeg; s++)
            {
                for (int i = 0; i < samplesPerSegment; i++)
                {
                    float t = i / (float)samplesPerSegment;
                    result[idx++] = EvaluateDerivative(cp, s, t, 1).normalized;
                }
            }
            result[total - 1] = EvaluateDerivative(cp, nSeg - 1, 0.999f, 1).normalized;
            return result;
        }

        public static float[] GetCurvature(Vector3[] controlPoints, int samplesPerSegment = 8, bool loop = false)
        {
            Vector3[] cp = PadControlPoints(controlPoints, loop);
            int nSeg = cp.Length - 3;
            int total = nSeg * samplesPerSegment + 1;
            var result = new float[total];

            int idx = 0;
            for (int s = 0; s < nSeg; s++)
            {
                for (int i = 0; i < samplesPerSegment; i++)
                {
                    float t = i / (float)samplesPerSegment;
                    Vector3 d1 = EvaluateDerivative(cp, s, t, 1);
                    Vector3 d2 = EvaluateDerivative(cp, s, t, 2);
                    float k = Vector3.Cross(d1, d2).magnitude / Mathf.Pow(d1.magnitude, 3);
                    result[idx++] = float.IsNaN(k) ? 0 : k;
                }
            }
            result[total - 1] = result[total - 2];
            return result;
        }

        // --- internal helpers ---
        private static Vector3[] PadControlPoints(Vector3[] input, bool loop)
        {
            int n = input.Length;
            if (loop)
            {
                var cp = new Vector3[n + 3];
                cp[0] = input[n - 2];
                cp[1] = input[n - 1];
                for (int i = 0; i < n; i++) cp[i + 2] = input[i];
                return cp;
            }
            else
            {
                var cp = new Vector3[n + 2];
                cp[0] = input[0] + (input[0] - input[1]) * 0.5f;
                for (int i = 0; i < n; i++) cp[i + 1] = input[i];
                cp[n + 1] = input[n - 1] + (input[n - 1] - input[n - 2]) * 0.5f;
                return cp;
            }
        }

        private static Vector3 EvaluatePosition(Vector3[] cp, int seg, float t)
        {
            float t2 = t * t;
            float t3 = t2 * t;
            Vector3 p0 = cp[seg], p1 = cp[seg+1], p2 = cp[seg+2], p3 = cp[seg+3];

            float b0 = (1f - t) * (1f - t) * (1f - t) / 6f;
            float b1 = (3f*t3 - 6f*t2 + 4f) / 6f;
            float b2 = (-3f*t3 + 3f*t2 + 3f*t + 1f) / 6f;
            float b3 = t3 / 6f;

            return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
        }

        private static Vector3 EvaluateDerivative(Vector3[] cp, int seg, float t, int order)
        {
            Vector3 p0 = cp[seg], p1 = cp[seg+1], p2 = cp[seg+2], p3 = cp[seg+3];
            if (order == 1)
            {
                // First derivative
                float t2 = t * t;
                float d0 = -0.5f * (1f - t) * (1f - t);
                float d1 = 1.5f * t2 - 2f * t;
                float d2 = -1.5f * t2 + t + 0.5f;
                float d3 = 0.5f * t2;
                return p0*d0 + p1*d1 + p2*d2 + p3*d3;
            }
            else
            {
                // Second derivative
                float d0 = (1f - t);
                float d1 = 3f*t - 2f;
                float d2 = -3f*t + 1f;
                float d3 = t;
                return p0*d0 + p1*d1 + p2*d2 + p3*d3;
            }
        }

        /// <summary>
        /// 在曲线上按弧长查找与参数 s（0~总长度）对应的点
        /// </summary>
        public static int FindIndexByArcLength(Vector3[] smoothPath, float s, out float t)
        {
            float acc = 0;
            for (int i = 0; i < smoothPath.Length - 1; i++)
            {
                float segLen = Vector3.Distance(smoothPath[i], smoothPath[i + 1]);
                if (acc + segLen >= s)
                {
                    t = (s - acc) / Mathf.Max(1e-6f, segLen);
                    return i;
                }
                acc += segLen;
            }
            t = 0;
            return smoothPath.Length - 2;
        }

        public static float TotalLength(Vector3[] path)
        {
            float s = 0;
            for (int i = 0; i < path.Length - 1; i++)
                s += Vector3.Distance(path[i], path[i+1]);
            return s;
        }
    }
}
