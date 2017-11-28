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
        bool releasedNow = false;
        LinkedList<CameraSpacePoint> recordedPoints = new LinkedList<CameraSpacePoint>();
        CameraSpacePoint prevPoint;

        public static float HAND_SHOULDER_Z_DELTA = 0.05f;
        public static float HAND_ELBOW_Y_DELTA = 0.01f;
        public static float TIME_OF_5_FRAMES = 0.17f;
        private double releasedAngle;
        private double releasedVelocity;

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
                            Joint shoulderRight = joints[JointType.ShoulderRight];

                            recording = inRecordingMode(joints) || recording;
                            releasedNow = isReleased(joints);

                            recodingModeLabel.Text = recording ? "TRUE" : "FALSE";
                            isReleasedLabel.Text = releasedNow ? "TRUE" : "FALSE";

                            if (recording)
                                recordedPoints.AddLast(rightHand.Position);

                            if (recording && releasedNow)
                            {
                                releasedAngle = calculateAngle();
                                releasedVelocity = calculateVelocity();
                                recordedPoints.Clear();
                                recording = false;

                                velocityLabel.Text = releasedVelocity.ToString("#.##");

                            }


                            recordedPoints.AddLast(rightHand.Position);

                            rightHandX.Text = rightHand.Position.X.ToString("#.##");
                            rightHandY.Text = rightHand.Position.Y.ToString("#.##");
                            rightHandZ.Text = rightHand.Position.Z.ToString("#.##");

                            rightHipX.Text = shoulderRight.Position.X.ToString("#.##");
                            rightHipY.Text = shoulderRight.Position.Y.ToString("#.##");
                            rightHipZ.Text = shoulderRight.Position.Z.ToString("#.##");

                            Console.Out.WriteLine($"Hand: {rightHand.Position.X} {rightHand.Position.Y} {rightHand.Position.Z}");
                            Console.Out.WriteLine($"Hip: {rightHip.Position.X} {rightHip.Position.Y} {rightHip.Position.Z}");
                        }
                    }
                }
            }
        }

        private double calculateVelocity()
        {
            double slope = 0;
            int len = recordedPoints.Count - 1;
            int numFrames = 5;
            int i;
            for (i = 5; i >= 0; i++)
            {
                if (len - i < 0)
                    break;

                slope += recordedPoints.ElementAt(len - i).Y / recordedPoints.ElementAt(len - i).Z;
            }

            slope /= numFrames - i;

            return slope / TIME_OF_5_FRAMES;
        }

        private double calculateAngle()
        {
            return 0;
        }

        private bool inRecordingMode(IReadOnlyDictionary<JointType, Joint> joints)
        {
            Joint rightHand = joints[JointType.HandRight];
            Joint rightElbow = joints[JointType.ElbowRight];
            Joint shoulderRight = joints[JointType.ShoulderRight];

            return rightHand.Position.Z > (shoulderRight.Position.Z + HAND_SHOULDER_Z_DELTA)
                && rightHand.Position.Y < (rightElbow.Position.Y + HAND_ELBOW_Y_DELTA);
        }

        private bool isReleased(IReadOnlyDictionary<JointType, Joint> joints)
        {
            Joint rightHand = joints[JointType.HandRight];
            Joint rightElbow = joints[JointType.ElbowRight];
            Joint shoulderRight = joints[JointType.ShoulderRight];
            //Joint hip = joints[JointType.HipRight];

            double dz = shoulderRight.Position.Z - rightHand.Position.Z;
            double dy = shoulderRight.Position.Y - rightHand.Position.Y;
            dzLabel.Text = dz.ToString("#.##");
            dyLabel.Text = dy.ToString("#.##");
            bool isReleased = dz >= dy;

            return isReleased;
        }

        private void label6_Click(object sender, EventArgs e)
        {

        }
    }
}
