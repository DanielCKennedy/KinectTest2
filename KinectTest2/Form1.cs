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
        public static int   NUM_FRAMES_TO_AVG = 5;

        private double releasedXDirection;
        private double releasedYDirection;
        private double releasedZDirection;
        private double releasedYVelocity;
        private double releasedXVelocity;
        private double releasedZVelocity;

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
                                CameraSpacePoint startingPoint = getStartingPoint();
                                CameraSpacePoint endingPoint = recordedPoints.ElementAt(recordedPoints.Count - 1);
                                releasedXVelocity = calculateVelocity(startingPoint.X, endingPoint.X);
                                releasedYVelocity = calculateVelocity(startingPoint.Y, endingPoint.Y);
                                releasedZVelocity = calculateVelocity(endingPoint.Z, startingPoint.Z);  //Flipped due to perspective being from Kinect
                                releasedXDirection = Math.Atan2(releasedXVelocity, releasedZVelocity);
                                releasedYDirection = Math.Atan2(releasedYVelocity, releasedZVelocity);

                                calculateLandingPosition(startingPoint, endingPoint);
                                recordedPoints.Clear();
                                recording = false;

                                xVelocityLabel.Text = releasedXVelocity.ToString("#.##") + " m/s";
                                zVelocityLabel.Text = releasedZVelocity.ToString("#.##") + " m/s";
                                yVelocityLabel.Text = releasedYVelocity.ToString("#.##") + " m/s";
                                xDirectionLabel.Text = releasedXDirection.ToString("#.##") + " degrees";
                                yDirectionLabel.Text = releasedYDirection.ToString("#.##") + " degrees";
                            }


                            recordedPoints.AddLast(rightHand.Position);

                            rightHandX.Text = rightHand.Position.X.ToString("#.##");
                            rightHandY.Text = rightHand.Position.Y.ToString("#.##");
                            rightHandZ.Text = rightHand.Position.Z.ToString("#.##");

                            rightHipX.Text = shoulderRight.Position.X.ToString("#.##");
                            rightHipY.Text = shoulderRight.Position.Y.ToString("#.##");
                            rightHipZ.Text = shoulderRight.Position.Z.ToString("#.##");

                            //Console.Out.WriteLine($"Hand: {rightHand.Position.X} {rightHand.Position.Y} {rightHand.Position.Z}");
                            //Console.Out.WriteLine($"Hip: {rightHip.Position.X} {rightHip.Position.Y} {rightHip.Position.Z}");
                        }
                    }
                }
            }
        }

        private void calculateLandingPosition(CameraSpacePoint startingPoint, CameraSpacePoint endingPoint)
        {
            double velocityYZ = Math.Sqrt(Math.Pow(releasedYVelocity, 2) + Math.Pow(releasedZVelocity, 2)); 
            double theta = Math.Atan((endingPoint.Y - startingPoint.Y) / (startingPoint.Z - endingPoint.Z));
            double acceleration = -9.8;
            double a = acceleration * .5;
            double b = velocityYZ * Math.Sin(theta);
            double c = startingPoint.Y;
            double time = (((-1) * b) - (Math.Sqrt(b*b - (4 * a * c)) / (2 * a)));
            if (time < 0)
                time = (((-1) * b) + (Math.Sqrt(b * b - (4 * a * c)) / (2 * a)));
            double distanceZ = velocityYZ * Math.Cos(theta) * time;
            double distanceX = releasedXVelocity * time;

            landingPosXLabel.Text = distanceX.ToString("#.##") + " meters away";
            landingPosZLabel.Text = distanceZ.ToString("#.##") + " meters away";
        }

        private CameraSpacePoint getStartingPoint()
        {
            int len = recordedPoints.Count - 1;
            CameraSpacePoint csp = recordedPoints.ElementAt(0);

            for (int i = NUM_FRAMES_TO_AVG; i >= 0 && len >= 0; i--)
            {
                if (len - i >= 0)
                {
                    csp = recordedPoints.ElementAt(len - i);
                    break;
                }
            }

            return csp;
        }

        private double calculateVelocity(double start, double end)
        {
            return (end - start) / TIME_OF_5_FRAMES;
        }

        private double calculateDirctionFromPoints(double startA, double startB, double endA, double endB)
        {
            return Math.Atan2((endA - startA), (endB - startB));
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

        private void Form1_Load(object sender, EventArgs e)
        {

        }
    }
}
