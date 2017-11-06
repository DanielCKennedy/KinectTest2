using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Microsoft.Kinect;

namespace KinectTest2
{
    public partial class Form1 : Form
    {
        KinectSensor kinectSensor = null;
        BodyFrameReader bodyFrameReader = null;
        Body[] bodies = null;

        bool recordReady = false;
        bool recording = false;
        LinkedList<CameraSpacePoint> recordedPoints = new LinkedList<CameraSpacePoint>();
        CameraSpacePoint prevPoint;

        public Form1()
        {
            InitializeComponent();
            initializeKinect();
        }

        public void initializeKinect()
        {
            kinectSensor = KinectSensor.GetDefault();

            if (kinectSensor != null)
            {
                // turn on kinect
                kinectSensor.Open();
            }

            bodyFrameReader = kinectSensor.BodyFrameSource.OpenReader();

            if (bodyFrameReader != null)
            {
                bodyFrameReader.FrameArrived += Reader_FrameArrived;
            }
        }

        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (bodies == null)
                    {
                        bodies = new Body[bodyFrame.BodyCount];
                    }
                    bodyFrame.GetAndRefreshBodyData(bodies);
                    dataReceived = true;
                }

                if (dataReceived)
                {
                    foreach (Body body in bodies)
                    {
                        if (body.IsTracked)
                        {
                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            Joint rightHand = joints[JointType.HandRight];
                            Joint rightHip = joints[JointType.HipRight];
                            Joint rightElbow = joints[JointType.ElbowRight];

                            rightHandX.Text = rightHand.Position.X.ToString("#.##");
                            rightHandY.Text = rightHand.Position.Y.ToString("#.##");
                            rightHandZ.Text = rightHand.Position.Z.ToString("#.##");

                            rightHipX.Text = rightHip.Position.X.ToString("#.##");
                            rightHipY.Text = rightHip.Position.Y.ToString("#.##");
                            rightHipZ.Text = rightHip.Position.Z.ToString("#.##");

                            Console.Out.WriteLine($"Hand: {rightHand.Position.X} {rightHand.Position.Y} {rightHand.Position.Z}");
                            Console.Out.WriteLine($"Hip: {rightHip.Position.X} {rightHip.Position.Y} {rightHip.Position.Z}");
                        }
                    }
                }
            }
        }
    }
}
