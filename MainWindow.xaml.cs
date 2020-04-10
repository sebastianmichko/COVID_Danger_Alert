//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.Covid_Danger_Alert
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Font size of face property text 
        /// </summary>
        private const double DrawTextFontSize = 10;

        /// <summary>
        /// Number of bodies currently tracked 
        /// </summary>
        private int bodies_currently_observed = 0;

        //Body/Pedestrian Class


        //List of Bodies

        //Overlay Stuff from Face Basics

        /// <summary>
        /// Text layout offset in X axis
        /// </summary>
        private const float TextLayoutOffsetX = -0.1f;//-1.0f;

        /// <summary>
        /// Text layout offset in Y axis
        /// </summary>
        private const float TextLayoutOffsetY = 0.15f;//0.25f;

            private Rect displayRect;

        System.Media.SoundPlayer alarm_1 = new System.Media.SoundPlayer(Path.Combine(Environment.CurrentDirectory, @"Audio\alarm_1.wav"));
        System.Media.SoundPlayer alarm_2 = new System.Media.SoundPlayer(Path.Combine(Environment.CurrentDirectory, @"Audio\alarm_2.wav"));

        //Alert Global Functons
        bool global_proximity_alert = false;
        bool global_contamination_alert = false;
        bool proximity_alarm_currently_playing = false;
        bool contamination_alarm_currently_playing = false;

        private List<int> proximity_alert_offenders = new List<int>();
        private List<int> contamination_alert_offenders = new List<int>();

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;
            this.displayRect = new Rect(0.0, 0.0, this.displayWidth, this.displayHeight);

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    //dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    dc.DrawRectangle(Brushes.Black, null, this.displayRect);

                    //Count Bodies Observed
                    bodies_currently_observed = 0;

                    //Check for alerts
                    global_contamination_alert = Contamination_Alert();
                    global_proximity_alert = Proximity_Alert();

                    //foreach (Body body in this.bodies)
                    for (int i = 0; i < this.bodies.Length; i++)
                    {
                        //Change pen color to red if that pedestrian is currently causing an alert
                        Pen drawPen = new Pen(Brushes.White, 6);
                        if (proximity_alert_offenders.Contains(i) || contamination_alert_offenders.Contains(i))
                        {
                            drawPen = new Pen(Brushes.Red, 6);
                        }                        

                        this.DrawFaceFrameResults(i, dc);

                        if (bodies[i].IsTracked)
                        {
                            //Increment Body Count
                            bodies_currently_observed++;

                            // draw face frame results

                            this.DrawClippedEdges(bodies[i], dc);

                            IReadOnlyDictionary<JointType, Joint> joints = bodies[i].Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);
                        }
                    }

                    //Play Audio Alerts

                    //Face-Touch Contamination Alert
                    if (global_contamination_alert)
                    {
                        if (!contamination_alarm_currently_playing && !proximity_alarm_currently_playing)
                        {
                            contamination_alarm_currently_playing = true;
                            alarm_1.PlayLooping();
                        }
                    }
                    else
                    {
                        if (contamination_alarm_currently_playing)
                        {
                            alarm_1.Stop();
                            contamination_alarm_currently_playing = false;
                        }
                    }

                    //Ped - Ped Proximity Alert - SHOULD BE THE MASTER ALARM IF THIS ONE IS ACTIVE THEN A FACE ALARM WON'T MATTER
                    if (global_proximity_alert)
                    {
                        if (!proximity_alarm_currently_playing)
                        {
                            contamination_alarm_currently_playing = false;
                            alarm_1.Stop();

                            proximity_alarm_currently_playing = true;
                            alarm_2.PlayLooping();
                        }
                    }
                    else
                    {
                        if (proximity_alarm_currently_playing)
                        {
                            alarm_2.Stop();
                            proximity_alarm_currently_playing = false;
                        }
                    }


                    //Display the Number of bodies tracked currently
                    dc.DrawText(
                                    new FormattedText(
                                    ("Number of Bodies Detected = " + bodies_currently_observed + "\nProximity Alert: " + global_proximity_alert + "\nFace Touch Alert: " + global_contamination_alert),
                                    CultureInfo.GetCultureInfo("en-us"),
                                    FlowDirection.LeftToRight,
                                    new Typeface("Georgia"),
                                    DrawTextFontSize,
                                    Brushes.White),
                                    new Point(displayWidth / 2, displayHeight - 50)
                                );

                    //Debug.Print("distance between Ped 1 & 2 = " + Find_Distance_Ped_Ped(0, 1));
                    //for(int i = 0; i < bodies.Length; i++)
                    //{
                    //    var head = bodies[i].Joints[JointType.Head];
                    //    var left_hand = bodies[i].Joints[JointType.HandLeft];
                    //    var right_hand = bodies[i].Joints[JointType.HandRight];

                    //    var head_position = head.Position;
                    //    var left_hand_position = left_hand.Position;
                    //    var right_hand_position = right_hand.Position;
                         

                    //    Debug.Print("#" + i + "/" + (bodies.Length - 1) + " " + (bodies[i].IsTracked) + " " + head_position.X + " " + head_position.Y + " " + head_position.Z + " " + Find_Distance(head, left_hand) + " " + Find_Distance(head, right_hand) );
                    //}

                        // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(this.displayRect);//new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        private double Find_Distance_Ped_Ped(int bodyIndex, int bodyIndex2)//Indexes are randomly assigned for some reason.
        {
            Body body1 = this.bodies[bodyIndex];
            Body body2 = this.bodies[bodyIndex2];

            if (body1.IsTracked && body2.IsTracked)//Failing
            {
                

                var head1 = body1.Joints[JointType.Head].Position;
                var head2 = body2.Joints[JointType.Head].Position;

                //Debug.Print(head1.X + " " + head1.Y + " " + head1.Z);
                //Debug.Print(head2.X + " " + head2.Y + " " + head2.Z);

                double distance = Math.Sqrt(Math.Pow( (head1.X - head2.X), 2) + Math.Pow( (head1.Y - head2.Y), 2) + Math.Pow( (head1.Z - head2.Z), 2));

                return Math.Round(distance, 2);
            }
            else return 0.0;
        }

        private double Find_Distance(Joint jointA, Joint jointB)//Possible better to take position instead of joint
        {
            //Body body1 = this.bodies[bodyIndex];
            //Body body2 = this.bodies[bodyIndex2];
            //Debug.Print("Tracking State" + jointA.TrackingState);
            //if (jointA.TrackingState.Equals(true))
            //{


                var jointAPosition = jointA.Position;
                var jointBPosition = jointB.Position;

                //Debug.Print(head1.X + " " + head1.Y + " " + head1.Z);
                //Debug.Print(head2.X + " " + head2.Y + " " + head2.Z);

                double distance = Math.Sqrt(Math.Pow((jointAPosition.X - jointBPosition.X), 2) + Math.Pow((jointAPosition.Y - jointBPosition.Y), 2) + Math.Pow((jointAPosition.Z - jointBPosition.Z), 2));

                return Math.Round(distance, 2);
            //}
            //else return 0.0;
        }

        private bool Proximity_Alert()//Indexes are randomly assigned for some reason.
        {
            bool ped_ped_in_proximity = false;
            proximity_alert_offenders.Clear();
            float min_distance = 1.0f;

            for(int x = 0; x < bodies.Length; ++x)
            {
                if(bodies[x].IsTracked)
                {
                    for (int y = 0; y < bodies.Length; ++y)
                    {
                        if(bodies[y].IsTracked && (x != y))
                        {
                            double temp_dist = Find_Distance_Ped_Ped(x, y);
                            if (temp_dist < min_distance)
                            {
                                ped_ped_in_proximity = true;
                                proximity_alert_offenders.Add(x);
                                proximity_alert_offenders.Add(y);
                            }
                            //Debug.Print("Distance = " + temp_dist);
                        }
                    }
                }
            }

            return ped_ped_in_proximity;
        }

        private bool Contamination_Alert()//Indexes are randomly assigned for some reason.
        {
            bool ped_face_touch = false;
            contamination_alert_offenders.Clear();
            float min_distance = 0.3f;

            for (int index = 0; index < bodies.Length; ++index)
            {
                if (bodies[index].IsTracked)
                {
                    var head = bodies[index].Joints[JointType.Head];
                    var left_hand = bodies[index].Joints[JointType.HandLeft];
                    var right_hand = bodies[index].Joints[JointType.HandRight];

                    if (Find_Distance(head, left_hand) < min_distance || Find_Distance(head, right_hand) < min_distance)
                    {
                        ped_face_touch = true;
                        contamination_alert_offenders.Add(index);
                    }
                }
            }

            return ped_face_touch;
        }

        /// <summary>
        /// Draws face frame results
        /// </summary>
        /// <param name="faceIndex">the index of the face frame corresponding to a specific body in the FOV</param>
        /// <param name="faceResult">container of all face frame results</param>
        /// <param name="drawingContext">drawing context to render to</param>
        private void DrawFaceFrameResults(int faceIndex, DrawingContext drawingContext)
        {
            // choose the brush based on the face index
            Brush drawingBrush = Brushes.White;

            string faceText = string.Empty;

            var head = bodies[faceIndex].Joints[JointType.Head];
            var left_hand = bodies[faceIndex].Joints[JointType.HandLeft];
            var right_hand = bodies[faceIndex].Joints[JointType.HandRight];

                faceText += "Pedestrian Index " + faceIndex + "\n" +
                            "Left Hand-Face Dist " + Find_Distance(head, left_hand) + "m\n" +
                            "Right Hand-Face Dist " + Find_Distance(head, right_hand)+  "m\n";

            // render the face property and face rotation information
            Point faceTextLayout;
            if (this.GetFaceTextPositionInColorSpace(faceIndex, out faceTextLayout))
            {
                drawingContext.DrawText(
                        new FormattedText(
                            faceText,
                            CultureInfo.GetCultureInfo("en-us"),
                            FlowDirection.LeftToRight,
                            new Typeface("Georgia"),
                            DrawTextFontSize,
                            drawingBrush),
                        faceTextLayout);
            }
        }

        /// <summary>
        /// Computes the face result text position by adding an offset to the corresponding 
        /// body's head joint in camera space and then by projecting it to screen space
        /// </summary>
        /// <param name="faceIndex">the index of the face frame corresponding to a specific body in the FOV</param>
        /// <param name="faceTextLayout">the text layout position in screen space</param>
        /// <returns>success or failure</returns>
        private bool GetFaceTextPositionInColorSpace(int faceIndex, out Point faceTextLayout)
        {
            faceTextLayout = new Point();
            bool isLayoutValid = false;

            Body body = this.bodies[faceIndex];
            if (body.IsTracked)
            {
                var headJoint = body.Joints[JointType.Head].Position;

                CameraSpacePoint textPoint = new CameraSpacePoint()
                {
                    X = headJoint.X + TextLayoutOffsetX,
                    Y = headJoint.Y + TextLayoutOffsetY,
                    Z = headJoint.Z
                };

                //ColorSpacePoint textPointInColor = this.coordinateMapper.MapCameraPointToColorSpace(textPoint);
                //faceTextLayout.X = textPointInColor.X;
                //faceTextLayout.Y = textPointInColor.Y;

                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(textPoint);
                faceTextLayout.X = depthSpacePoint.X;
                faceTextLayout.Y = depthSpacePoint.Y;


                isLayoutValid = true;
            }

            return isLayoutValid;
        }
    }
}