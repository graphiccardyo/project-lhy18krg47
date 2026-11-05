using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

// Small software 3D renderer. Rotation uses sensor-to-world quaternions (w,x,y,z).
public sealed class MpuAttitudeView : Control
{
    private double w = 1, x, y, z, yawReference;
    private bool hasPose;
    public int RenderCount { get; private set; }
    public double[] LastQuaternion { get { return new double[] { w, x, y, z }; } }
    private struct V
    {
        public double X, Y, Z;
        public V(double x, double y, double z) { X = x; Y = y; Z = z; }
    }
    private sealed class Face
    {
        public V[] Points;
        public Color Color;
        public double Depth;
    }
    public MpuAttitudeView()
    {
        DoubleBuffered = true;
        ResizeRedraw = true;
        BackColor = Color.FromArgb(250, 252, 255);
    }
    public void SetOrientation(double qw, double qx, double qy, double qz)
    {
        double norm = Math.Sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (double.IsNaN(norm) || double.IsInfinity(norm) || norm < 0.5) return;
        w = qw / norm; x = qx / norm; y = qy / norm; z = qz / norm;
        hasPose = true;
        Invalidate();
    }
    public void SetYawReference(double degrees) { yawReference = degrees * Math.PI / 180; Invalidate(); }
    public void SetDisconnected() { hasPose = false; Invalidate(); }
    public double[] GetDisplayVector(double vx, double vy, double vz)
    {
        V v = Rotate(new V(vx, vy, vz));
        return new double[] { v.X, v.Y, v.Z };
    }
    public static double[] RotateVector(double w, double x, double y, double z, double vx, double vy, double vz)
    {
        double n = Math.Sqrt(w * w + x * x + y * y + z * z);
        if (n < 1e-12) throw new ArgumentException("Zero quaternion");
        w /= n; x /= n; y /= n; z /= n;
        double tx = 2 * (y * vz - z * vy), ty = 2 * (z * vx - x * vz), tz = 2 * (x * vy - y * vx);
        return new double[] { vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz, vz + w * tz + x * ty - y * tx };
    }
    private V Rotate(V v)
    {
        double[] r = RotateVector(w, x, y, z, v.X, v.Y, v.Z);
        double c = Math.Cos(yawReference), s = Math.Sin(yawReference);
        return new V(c * r[0] + s * r[1], -s * r[0] + c * r[1], r[2]);
    }
    private double Depth(V v) { return 7.0 - (0.503 * v.X - 0.704 * v.Y + 0.503 * v.Z); }
    private PointF Project(V v)
    {
        double right = 0.814 * v.X + 0.581 * v.Y;
        double up = -0.292 * v.X + 0.410 * v.Y + 0.864 * v.Z;
        double scale = Height * 2.0 / Math.Max(1.0, Depth(v));
        return new PointF((float)(Width * 0.50 + right * scale), (float)(Height * 0.51 - up * scale));
    }
    private void AddBox(List<Face> faces, double sx, double sy, double z0, double z1, Color top, Color side)
    {
        V[] p = new V[] { new V(-sx,-sy,z0), new V(sx,-sy,z0), new V(sx,sy,z0), new V(-sx,sy,z0),
                          new V(-sx,-sy,z1), new V(sx,-sy,z1), new V(sx,sy,z1), new V(-sx,sy,z1) };
        for (int i = 0; i < p.Length; i++) p[i] = Rotate(p[i]);
        int[][] indexes = new int[][] { new int[]{0,1,2,3}, new int[]{4,5,6,7}, new int[]{0,1,5,4}, new int[]{1,2,6,5}, new int[]{2,3,7,6}, new int[]{3,0,4,7} };
        for (int i = 0; i < indexes.Length; i++)
        {
            Face face = new Face(); face.Points = new V[4]; face.Color = i == 1 ? top : side;
            for (int j = 0; j < 4; j++) { face.Points[j] = p[indexes[i][j]]; face.Depth += Depth(face.Points[j]) / 4; }
            faces.Add(face);
        }
    }
    private void Axis(Graphics g, V end, Color color, string label, bool body)
    {
        V origin = new V(0, 0, 0);
        if (body) end = Rotate(end);
        using (Pen pen = new Pen(color, body ? 2.8f : 1.2f))
        using (AdjustableArrowCap cap = new AdjustableArrowCap(4, 5))
        using (Brush brush = new SolidBrush(color))
        using (Font font = new Font("Segoe UI", 10, FontStyle.Bold))
        {
            pen.CustomEndCap = cap;
            g.DrawLine(pen, Project(origin), Project(end));
            PointF point = Project(end);
            g.DrawString(label, font, brush, point.X + 5, point.Y - 9);
        }
    }
    protected override void OnPaint(PaintEventArgs e)
    {
        base.OnPaint(e);
        RenderCount++;
        Graphics g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        using (Pen grid = new Pen(Color.FromArgb(224, 231, 239)))
        {
            for (int i = -4; i <= 4; i++)
            {
                double d = i * 0.65;
                g.DrawLine(grid, Project(new V(-2.6, d, -1.0)), Project(new V(2.6, d, -1.0)));
                g.DrawLine(grid, Project(new V(d, -2.6, -1.0)), Project(new V(d, 2.6, -1.0)));
            }
        }
        Axis(g, new V(2.6, 0, 0), Color.Silver, "Xw", false);
        Axis(g, new V(0, 2.6, 0), Color.Silver, "Yw", false);
        Axis(g, new V(0, 0, 2.1), Color.Silver, "Zw", false);
        if (hasPose)
        {
            List<Face> faces = new List<Face>();
            AddBox(faces, 1.45, 0.92, -0.07, 0.07, Color.FromArgb(24, 139, 127), Color.FromArgb(15, 92, 89));
            AddBox(faces, 0.35, 0.35, 0.07, 0.28, Color.FromArgb(62, 72, 87), Color.FromArgb(39, 47, 59));
            faces.Sort(delegate(Face a, Face b) { return b.Depth.CompareTo(a.Depth); });
            using (Pen outline = new Pen(Color.FromArgb(50, 70, 77), 1.0f))
            {
                foreach (Face face in faces)
                {
                    PointF[] points = new PointF[face.Points.Length];
                    for (int i = 0; i < points.Length; i++) points[i] = Project(face.Points[i]);
                    using (Brush brush = new SolidBrush(face.Color)) g.FillPolygon(brush, points);
                    g.DrawPolygon(outline, points);
                }
            }
            Axis(g, new V(2.0, 0, 0), Color.FromArgb(202, 62, 58), "+X", true);
            Axis(g, new V(0, 1.65, 0), Color.FromArgb(31, 147, 71), "+Y", true);
            Axis(g, new V(0, 0, 1.75), Color.FromArgb(45, 104, 209), "+Z", true);
        }
        using (Font font = new Font("Segoe UI", 9))
        using (Brush text = new SolidBrush(Color.FromArgb(76, 91, 110)))
        {
            g.DrawString("BODY: X red / Y green / Z blue    |    WORLD: gray axes, Z up", font, text, 15, 12);
            if (!hasPose) g.DrawString("Waiting for orientation data...", font, text, Width / 2 - 90, Height / 2);
            g.DrawString("Orientation only - no position tracking", font, text, 15, Height - 27);
        }
    }
}
