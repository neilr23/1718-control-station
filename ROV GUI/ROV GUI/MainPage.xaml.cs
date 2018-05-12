using System;
using System.Text;
using System.Threading;
using System.Collections;
using System.Threading.Tasks;
using System.Linq;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Drawing;

using Microsoft.Win32.SafeHandles;

using Windows.Devices.SerialCommunication;
using Windows.Devices.Enumeration;
using Windows.Storage.Streams;
using Windows.Gaming.Input;

using Windows.UI.Core;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Media.Imaging;

//using DS4Windows;




// The Blank Page item template is documented at https://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace ROV_GUI
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    #pragma warning disable CS4014 // Because this call is not awaited, execution of the current method continues before the call is completed
    public sealed partial class MainPage : Page
    {
        //GUI
        private Timer updateTimer;
        private double voltage;
        private double motorRPM1;
        private double motorRPM2;
        private double motorRPM3;
        private double motorRPM4;
        private double motorRPM5;
        private double motorRPM6;
        private double speedV;
        private double accelerationV;
        private double manipulator1;
        private double manipulator2;
        //COMMs
        private SerialDevice UartPort;
        private DataReader DataReaderObject = null;
        private DataWriter DataWriterObject;
        private CancellationTokenSource ReadCancellationTokenSource;
        private byte[] modbusRegisters;
        private byte[] manipRegisters;
        private String myCom = "COM0";
        private uint baudRate = 250000;
        private Boolean connected;
        private Boolean helperC;
        //Controller
        private RawGameController[] controllers;
        private Timer pilotTimer;
        private Boolean controller1;
        private Boolean controller2;
        private int buttonCount;
        private int axisCount;
        private Boolean[] buttonOneStates;
        private Double[] axisOneStates;
        private Boolean[] buttonTwoStates;
        private Double[] axisTwoStates;
        /////////////////////////////////////////////////////////////////////////////Initialize
        public MainPage()
        {
            this.InitializeComponent();
            baudInput.Text = "" + baudRate;
            //Controller
            AutoResetEvent AutoEvent = new AutoResetEvent(true);
            controllers = new RawGameController[2];
            for (int a = 0; a < RawGameController.RawGameControllers.Count; a++)
            {
                controllers[a] = RawGameController.RawGameControllers[a];
            }
            pilotTimer = new Timer(new TimerCallback(checkAndSend), AutoEvent, 0, 1); //Timer to do things every fucking millisecond lmaooo
            updateTimer = new Timer(new TimerCallback(UpdateGUI), AutoEvent, 0, 1);
            //COMMs
            modbusRegisters = new byte[28];
            manipRegisters = new byte[4];
            Initialize(baudRate, myCom);
        }
        /////////////////////////////////////////////////////////////////////////////GUI interactions
        private void UpdateGUI(object state)
        {
            UpdatePrivateValues();
            if (connected != helperC)
            {
                if (connected)
                {
                    Windows.ApplicationModel.Core.CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
                     {
                         comSlider.IsEnabled = false;
                         baudInput.IsEnabled = false;
                         ConnectedImg.Source = new BitmapImage(new Uri("ms-appx:///Assets/connection.png"));
                         if (controller1)
                             Controller1Img.Source = new BitmapImage(new Uri("ms-appx:///Assets/ps4_controller1.jpg"));
                         else
                             Controller1Img.Source = new BitmapImage(new Uri("ms-appx:///Assets/Square44x44Logo.png"));
                         if (controller2)
                             Controller2Img.Source = new BitmapImage(new Uri("ms-appx:///Assets/ps4_controller1.jpg"));
                         else
                             Controller2Img.Source = new BitmapImage(new Uri("ms-appx:///Assets/Square44x44Logo.png"));
                         connectionStatus.Text = "True";
                         comPortId.Text = myCom;
                         baudSpeedId.Text = ""+baudRate;
                         voltageLabel.Text = "Failure";
                         rpm1Label.Text = "Failure";
                         rpm2Label.Text = "Failure";
                         rpm3Label.Text = "Failure";
                         rpm4Label.Text = "Failure";
                         rpm5Label.Text = "Failure";
                         rpm6Label.Text = "Failure";
                         speedLabel.Text = "Failure";
                         accelLabel.Text = "Failure";
                         mani1Label.Text = "Failure";
                         mani2Label.Text = "Failure";
                     });
                    helperC = connected;
                }
                else
                {
                    Windows.ApplicationModel.Core.CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
                     {
                         comSlider.IsEnabled = true;
                         baudInput.IsEnabled = true;
                         ConnectedImg.Source = new BitmapImage(new Uri("ms-appx:///Assets/Square44x44Logo.png"));
                         if(controller1)
                            Controller1Img.Source = new BitmapImage(new Uri("ms-appx:///Assets/ps4_controller1.jpg"));
                         else
                            Controller1Img.Source = new BitmapImage(new Uri("ms-appx:///Assets/Square44x44Logo.png"));
                         if(controller2)
                            Controller2Img.Source = new BitmapImage(new Uri("ms-appx:///Assets/ps4_controller1.jpg"));
                         else
                            Controller2Img.Source = new BitmapImage(new Uri("ms-appx:///Assets/Square44x44Logo.png"));
                         connectionStatus.Text = "False";
                         comPortId.Text = "N/A";
                         baudSpeedId.Text = "N/A";
                         voltageLabel.Text = "Failure";
                         rpm1Label.Text = "Failure";
                         rpm2Label.Text = "Failure";
                         rpm3Label.Text = "Failure";
                         rpm4Label.Text = "Failure";
                         rpm5Label.Text = "Failure";
                         rpm6Label.Text = "Failure";
                         speedLabel.Text = "Failure";
                         accelLabel.Text = "Failure";
                         mani1Label.Text = "Failure";
                         mani2Label.Text = "Failure";
                     });
                    helperC = connected;
                }
            }
        }

        private void UpdatePrivateValues()
        {
            Windows.ApplicationModel.Core.CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            {
                int num;
                myCom = "COM" + comSlider.Value;
                if(int.TryParse(baudInput.Text, out num))
                {
                    baudRate = UInt32.Parse(baudInput.Text);
                }
                else
                {
                    ContentDialog failedtoParseDialog = new ContentDialog
                    {
                        Title = "Failed to Parse Baud Rate",
                        Content = baudInput.Text + " is not a number",
                        CloseButtonText = "Ok"
                    };
                }
                //Needs error exception TwT
            });
        }

        private void HorizontalButton_Click(object sender, RoutedEventArgs e)
        {

        }

        private void VerticalButton_Click(object sender, RoutedEventArgs e)
        {

        }

        private void Slider_ValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            Slider slider = sender as Slider;
            if (slider != null)
            {
                myCom = "COM" + slider.Value;
            }
        }
        /////////////////////////////////////////////////////////////////////////////Backend
        //public event EventHandler<RawGameController> RawGameControllerAdded;
        static void RawGameControllerAdded(object sender, RawGameController args)
        {
            RawGameControllers.Add(args);
        }
        public static ArrayList RawGameControllers{ get; set; }
        
        private void checkAndSend(object state)//We send things here
        {
            string deploy = "";
            try
            {
                if (!(RawGameControllers.Count == 0))
                {
                    controllers[0].GetCurrentReading(buttonOneStates, null, axisOneStates);
                    controllers[1].GetCurrentReading(buttonTwoStates, null, axisTwoStates);

                    
                    for (int a = 0; a < buttonOneStates.Length; a++)
                    {
                        deploy += "Button " + a + ": " + buttonOneStates[a] + " |";
                    }
                    PilotOne.Text = deploy;
                    if (buttonOneStates[0])
                    {
                        //cyka blyAT
                    }
                    else if (buttonOneStates[1])
                    {

                    }
                    else if (buttonOneStates[2])
                    {

                    }
                    else if (buttonOneStates[3])
                    {

                    }
                    else if (buttonOneStates[4])
                    {

                    }
                    else if (buttonOneStates[5])
                    {

                    }
                    else if (buttonOneStates[6])
                    {

                    }
                    else if (buttonOneStates[7])
                    {

                    }
                    else if (buttonOneStates[8])
                    {

                    }
                    else if (buttonOneStates[9])
                    {

                    }
                    else if (buttonOneStates[10])
                    {

                    }
                    else if (buttonOneStates[11])
                    {

                    }
                    if (buttonTwoStates[0])
                    {

                    }
                    else if (buttonTwoStates[1])
                    {

                    }
                    else if (buttonTwoStates[2])
                    {

                    }
                    else if (buttonTwoStates[3])
                    {

                    }
                    else if (buttonTwoStates[4])
                    {

                    }
                    else if (buttonTwoStates[5])
                    {

                    }
                    else if (buttonTwoStates[6])
                    {

                    }
                    else if (buttonTwoStates[7])
                    {

                    }
                    else if (buttonTwoStates[8])
                    {

                    }
                    else if (buttonTwoStates[9])
                    {

                    }
                    else if (buttonTwoStates[10])
                    {

                    }
                    else if (buttonTwoStates[11])
                    {

                    }
                    //a whole shit ton of if statements of what happens when buttons are pressed, put results into modbusRegisters and manipRegisters
                    //See the arduino code for explanations of what each place in modbusRegisters represents. 
                    SendBytes(modbusRegisters, manipRegisters);
                }
            }
            catch (Exception ex)
            {
                /*connected = false;
                ContentDialog failedtoConnectControllersDialog = new ContentDialog
                {
                    Title = "Boi where are the controllers?",
                    Content = ex.ToString(),
                    CloseButtonText = "OH SUGAR"
                };

                ContentDialogResult result = await failedtoConnectControllersDialog.ShowAsync();*/
            }
        }

        
        public async Task Initialize(uint BaudRate, String myCom)     //NOTE - THIS IS AN ASYNC METHOD!
        {
            try
            {
                string aqs = SerialDevice.GetDeviceSelector(myCom);
                var dis = await DeviceInformation.FindAllAsync(aqs);
                UartPort = await SerialDevice.FromIdAsync(dis[0].Id);

                //Configure serial settings
                UartPort.WriteTimeout = TimeSpan.FromMilliseconds(1000);    //mS before a time-out occurs when a write operation does not finish (default=InfiniteTimeout).
                UartPort.ReadTimeout = TimeSpan.FromMilliseconds(1000);     //mS before a time-out occurs when a read operation does not finish (default=InfiniteTimeout).
                UartPort.BaudRate = 9600;
                UartPort.Parity = SerialParity.None;
                UartPort.StopBits = SerialStopBitCount.One;
                UartPort.DataBits = 8;

                DataReaderObject = new DataReader(UartPort.InputStream);
                DataReaderObject.InputStreamOptions = InputStreamOptions.Partial;
                DataWriterObject = new DataWriter(UartPort.OutputStream);

                connected = true;

                StartReceive();
            }
            catch (Exception ex)
            {
                connected = false;
                ContentDialog failedtoConnectDialog = new ContentDialog
                {
                    Title = "Failed to Connect",
                    Content = ex.ToString(),
                    CloseButtonText = "Ok"
                };

                ContentDialogResult result = await failedtoConnectDialog.ShowAsync();
            }
        }

        public async void StartReceive()
        {

            ReadCancellationTokenSource = new CancellationTokenSource();

            while (true)
            {
                await Listen();
                if ((ReadCancellationTokenSource.Token.IsCancellationRequested) || (UartPort == null))
                    break;
            }
        }

        //LISTEN FOR NEXT RECEIVE
        private async Task Listen()
        {
            const int NUMBER_OF_BYTES_TO_RECEIVE = 1;           //<<<<<SET THE NUMBER OF BYTES YOU WANT TO WAIT FOR

            Task<UInt32> loadAsyncTask;
            byte[] ReceiveData;
            UInt32 bytesRead;

            try
            {
                if (UartPort != null)
                {
                    while (true)
                    {
                        //###### WINDOWS IoT MEMORY LEAK BUG 2017-03 - USING CancellationToken WITH LoadAsync() CAUSES A BAD MEMORY LEAK.  WORKAROUND IS
                        //TO BUILD RELEASE WITHOUT USING THE .NET NATIVE TOOLCHAIN OR TO NOT USE A CancellationToken IN THE CALL #####
                        //bytesRead = await DataReaderObject.LoadAsync(NUMBER_OF_BYTES_TO_RECEIVE).AsTask(ReadCancellationTokenSource.Token);	//Wait until buffer is full
                        bytesRead = await DataReaderObject.LoadAsync(NUMBER_OF_BYTES_TO_RECEIVE).AsTask();  //Wait until buffer is full

                        if ((ReadCancellationTokenSource.Token.IsCancellationRequested) || (UartPort == null))
                            break;

                        if (bytesRead > 0)
                        {
                            ReceiveData = new byte[NUMBER_OF_BYTES_TO_RECEIVE];
                            DataReaderObject.ReadBytes(ReceiveData);

                            foreach (byte Data in ReceiveData)
                            {

                                //Do something with it

                            } //foreach (byte Data in ReceiveData)

                        }

                    }
                }
            }
            catch (Exception e)
            {
                //We will get here often if the USB serial cable is removed so reset ready for a new connection (otherwise a never ending error occurs)
                if (ReadCancellationTokenSource != null)
                    ReadCancellationTokenSource.Cancel();
                System.Diagnostics.Debug.WriteLine("UART ReadAsync Exception: {0}", e.Message);
            }
        }

        public async void SendBytes(byte[] TxData, byte[] TxData2)
        {
            try
            {
                //Send data to UART
                DataWriterObject.WriteBytes(TxData);
                DataWriterObject.WriteBytes(TxData2);
                await DataWriterObject.StoreAsync();
            }
            catch (Exception ex)
            {
                connected = false;
                ContentDialog failedtoSendDialog = new ContentDialog
                {
                    Title = "Failed to Send Data",
                    Content = ex.ToString(),
                    CloseButtonText = "Ok"
                };

                ContentDialogResult result = await failedtoSendDialog.ShowAsync();
            }
        }

        private void ConnectButton_Click(object sender, RoutedEventArgs e)
        {
            Initialize(baudRate, myCom);
        }
    }
    /*//DS4STATE
    public class DS4State
    {
        public DateTime ReportTimeStamp;
        public bool Square, Triangle, Circle, Cross;
        public bool DpadUp, DpadDown, DpadLeft, DpadRight;
        public bool L1, L3, R1, R3;
        public bool Share, Options, PS, Touch1, Touch2, TouchButton, TouchRight,
            TouchLeft, Touch1Finger, Touch2Fingers;
        public byte Touch1Identifier, Touch2Identifier;
        public byte LX, RX, LY, RY, L2, R2;
        public byte FrameCounter; // 0, 1, 2...62, 63, 0....
        public byte TouchPacketCounter; // we break these out automatically
        public byte Battery; // 0 for charging, 10/20/30/40/50/60/70/80/90/100 for percentage of full
        public double LSAngle; // Calculated bearing of the LS X,Y coordinates
        public double RSAngle; // Calculated bearing of the RS X,Y coordinates
        public double LSAngleRad; // Calculated bearing of the LS X,Y coordinates (in radians)
        public double RSAngleRad; // Calculated bearing of the RS X,Y coordinates (in radians)
        public double LXUnit;
        public double LYUnit;
        public double RXUnit;
        public double RYUnit;
        public double elapsedTime = 0.0;
        public SixAxis Motion = null;
        public static readonly int DEFAULT_AXISDIR_VALUE = 127;

        public DS4State()
        {
            Square = Triangle = Circle = Cross = false;
            DpadUp = DpadDown = DpadLeft = DpadRight = false;
            L1 = L3 = R1 = R3 = false;
            Share = Options = PS = Touch1 = Touch2 = TouchButton = TouchRight = TouchLeft = false;
            Touch1Finger = Touch2Fingers = false;
            LX = RX = LY = RY = 127;
            L2 = R2 = 0;
            FrameCounter = 255; // only actually has 6 bits, so this is a null indicator
            TouchPacketCounter = 255; // 8 bits, no great junk value
            Battery = 0;
            LSAngle = 0.0;
            LSAngleRad = 0.0;
            RSAngle = 0.0;
            RSAngleRad = 0.0;
            LXUnit = 0.0;
            LYUnit = 0.0;
            RXUnit = 0.0;
            RYUnit = 0.0;
            elapsedTime = 0.0;
            Motion = new SixAxis(0, 0, 0, 0, 0, 0, 0.0);
        }

        public DS4State(DS4State state)
        {
            ReportTimeStamp = state.ReportTimeStamp;
            Square = state.Square;
            Triangle = state.Triangle;
            Circle = state.Circle;
            Cross = state.Cross;
            DpadUp = state.DpadUp;
            DpadDown = state.DpadDown;
            DpadLeft = state.DpadLeft;
            DpadRight = state.DpadRight;
            L1 = state.L1;
            L2 = state.L2;
            L3 = state.L3;
            R1 = state.R1;
            R2 = state.R2;
            R3 = state.R3;
            Share = state.Share;
            Options = state.Options;
            PS = state.PS;
            Touch1 = state.Touch1;
            TouchRight = state.TouchRight;
            TouchLeft = state.TouchLeft;
            Touch1Identifier = state.Touch1Identifier;
            Touch2 = state.Touch2;
            Touch2Identifier = state.Touch2Identifier;
            TouchButton = state.TouchButton;
            TouchPacketCounter = state.TouchPacketCounter;
            Touch1Finger = state.Touch1Finger;
            Touch2Fingers = state.Touch2Fingers;
            LX = state.LX;
            RX = state.RX;
            LY = state.LY;
            RY = state.RY;
            FrameCounter = state.FrameCounter;
            Battery = state.Battery;
            LSAngle = state.LSAngle;
            LSAngleRad = state.LSAngleRad;
            RSAngle = state.RSAngle;
            RSAngleRad = state.RSAngleRad;
            LXUnit = state.LXUnit;
            LYUnit = state.LYUnit;
            RXUnit = state.RXUnit;
            RYUnit = state.RYUnit;
            elapsedTime = state.elapsedTime;
            Motion = state.Motion;
        }

        public DS4State Clone()
        {
            return new DS4State(this);
        }

        public void CopyTo(DS4State state)
        {
            state.ReportTimeStamp = ReportTimeStamp;
            state.Square = Square;
            state.Triangle = Triangle;
            state.Circle = Circle;
            state.Cross = Cross;
            state.DpadUp = DpadUp;
            state.DpadDown = DpadDown;
            state.DpadLeft = DpadLeft;
            state.DpadRight = DpadRight;
            state.L1 = L1;
            state.L2 = L2;
            state.L3 = L3;
            state.R1 = R1;
            state.R2 = R2;
            state.R3 = R3;
            state.Share = Share;
            state.Options = Options;
            state.PS = PS;
            state.Touch1 = Touch1;
            state.Touch1Identifier = Touch1Identifier;
            state.Touch2 = Touch2;
            state.Touch2Identifier = Touch2Identifier;
            state.TouchLeft = TouchLeft;
            state.TouchRight = TouchRight;
            state.TouchButton = TouchButton;
            state.TouchPacketCounter = TouchPacketCounter;
            state.Touch1Finger = Touch1Finger;
            state.Touch2Fingers = Touch2Fingers;
            state.LX = LX;
            state.RX = RX;
            state.LY = LY;
            state.RY = RY;
            state.FrameCounter = FrameCounter;
            state.Battery = Battery;
            state.LSAngle = LSAngle;
            state.LSAngleRad = LSAngleRad;
            state.RSAngle = RSAngle;
            state.RSAngleRad = RSAngleRad;
            state.LXUnit = LXUnit;
            state.LYUnit = LYUnit;
            state.RXUnit = RXUnit;
            state.RYUnit = RYUnit;
            state.elapsedTime = elapsedTime;
            state.Motion = Motion;
        }

        public void calculateStickAngles()
        {
            double lsangle = Math.Atan2((LX - 127), -(LY - 127));
            LSAngleRad = lsangle;
            lsangle = (lsangle >= 0 ? lsangle : (2 * Math.PI + lsangle)) * 180 / Math.PI;
            LSAngle = lsangle;
            LXUnit = Math.Abs(Math.Cos(LSAngleRad));
            LYUnit = Math.Abs(Math.Sin(LSAngleRad));

            double rsangle = Math.Atan2((RX - 127), -(RY - 127));
            RSAngleRad = rsangle;
            rsangle = (rsangle >= 0 ? rsangle : (2 * Math.PI + rsangle)) * 180 / Math.PI;
            RSAngle = rsangle;
            RXUnit = Math.Abs(Math.Cos(RSAngleRad));
            RYUnit = Math.Abs(Math.Sin(LSAngleRad));
        }

        public void rotateLSCoordinates(double rotation)
        {
            double sinAngle = Math.Sin(rotation), cosAngle = Math.Cos(rotation);
            double tempLX = LX - 127.5, tempLY = LY - 127.5;
            LX = (Byte)(Global.Clamp(-127.5, (tempLX * cosAngle - tempLY * sinAngle), 127.5) + 127.5);
            LY = (Byte)(Global.Clamp(-127.5, (tempLX * sinAngle + tempLY * cosAngle), 127.5) + 127.5);
        }

        public void rotateRSCoordinates(double rotation)
        {
            double sinAngle = Math.Sin(rotation), cosAngle = Math.Cos(rotation);
            double tempRX = RX - 127.5, tempRY = RY - 127.5;
            RX = (Byte)(Global.Clamp(-127.5, (tempRX * cosAngle - tempRY * sinAngle), 127.5) + 127.5);
            RY = (Byte)(Global.Clamp(-127.5, (tempRX * sinAngle + tempRY * cosAngle), 127.5) + 127.5);
        }
    }
    //DS4DEVICE
    public struct DS4Color
    {
        public byte red;
        public byte green;
        public byte blue;
        public DS4Color(Color c)
        {
            red = c.R;
            green = c.G;
            blue = c.B;
        }

        public DS4Color(byte r, byte g, byte b)
        {
            red = r;
            green = g;
            blue = b;
        }

        public override bool Equals(object obj)
        {
            if (obj is DS4Color)
            {
                DS4Color dsc = ((DS4Color)obj);
                return (this.red == dsc.red && this.green == dsc.green && this.blue == dsc.blue);
            }
            else
                return false;
        }

        public Color ToColor => Color.FromArgb(red, green, blue);
        public Color ToColorA
        {
            get
            {
                byte alphacolor = Math.Max(red, Math.Max(green, blue));
                Color reg = Color.FromArgb(red, green, blue);
                Color full = HuetoRGB(reg.GetHue(), reg.GetBrightness(), reg);
                return Color.FromArgb((alphacolor > 205 ? 255 : (alphacolor + 50)), full);
            }
        }

        private Color HuetoRGB(float hue, float light, Color rgb)
        {
            float L = (float)Math.Max(.5, light);
            float C = (1 - Math.Abs(2 * L - 1));
            float X = (C * (1 - Math.Abs((hue / 60) % 2 - 1)));
            float m = L - C / 2;
            float R = 0, G = 0, B = 0;
            if (light == 1) return Color.White;
            else if (rgb.R == rgb.G && rgb.G == rgb.B) return Color.White;
            else if (0 <= hue && hue < 60) { R = C; G = X; }
            else if (60 <= hue && hue < 120) { R = X; G = C; }
            else if (120 <= hue && hue < 180) { G = C; B = X; }
            else if (180 <= hue && hue < 240) { G = X; B = C; }
            else if (240 <= hue && hue < 300) { R = X; B = C; }
            else if (300 <= hue && hue < 360) { R = C; B = X; }
            return Color.FromArgb((int)((R + m) * 255), (int)((G + m) * 255), (int)((B + m) * 255));
        }

        public static bool TryParse(string value, ref DS4Color ds4color)
        {
            try
            {
                string[] ss = value.Split(',');
                return byte.TryParse(ss[0], out ds4color.red) && byte.TryParse(ss[1], out ds4color.green) && byte.TryParse(ss[2], out ds4color.blue);
            }
            catch { return false; }
        }

        public override string ToString() => $"Red: {red} Green: {green} Blue: {blue}";
    }

    public enum ConnectionType : byte { BT, SONYWA, USB }; // Prioritize Bluetooth when both BT and USB are connected.

    public struct DS4HapticState
    {
        public DS4Color LightBarColor;
        public bool LightBarExplicitlyOff;
        public byte LightBarFlashDurationOn, LightBarFlashDurationOff;
        public byte RumbleMotorStrengthLeftHeavySlow, RumbleMotorStrengthRightLightFast;
        public bool RumbleMotorsExplicitlyOff;

        public bool IsLightBarSet()
        {
            return LightBarExplicitlyOff || LightBarColor.red != 0 || LightBarColor.green != 0 || LightBarColor.blue != 0;
        }

        public bool IsRumbleSet()
        {
            return RumbleMotorsExplicitlyOff || RumbleMotorStrengthLeftHeavySlow != 0 || RumbleMotorStrengthRightLightFast != 0;
        }
    }

    public class DS4Device
    {
        internal const int BT_OUTPUT_REPORT_LENGTH = 78;
        internal const int BT_INPUT_REPORT_LENGTH = 547;
        // Use large value for worst case scenario
        internal const int READ_STREAM_TIMEOUT = 3000;
        // Isolated BT report can have latency as high as 15 ms
        // due to hardware.
        internal const int WARN_INTERVAL_BT = 20;
        internal const int WARN_INTERVAL_USB = 10;
        // Maximum values for battery level when no USB cable is connected
        // and when a USB cable is connected
        internal const int BATTERY_MAX = 8;
        internal const int BATTERY_MAX_USB = 11;
        public const string blankSerial = "00:00:00:00:00:00";
        private HidDevice hDevice;
        private string Mac;
        private DS4State cState = new DS4State();
        private DS4State pState = new DS4State();
        private ConnectionType conType;
        private byte[] accel = new byte[6];
        private byte[] gyro = new byte[6];
        private byte[] inputReport;
        private byte[] btInputReport = null;
        private byte[] outReportBuffer, outputReport;
        private readonly DS4Touchpad touchpad = null;
        private readonly DS4SixAxis sixAxis = null;
        private byte rightLightFastRumble;
        private byte leftHeavySlowRumble;
        private DS4Color ligtBarColor;
        private byte ledFlashOn, ledFlashOff;
        private Thread ds4Input, ds4Output;
        private int battery;
        private DS4Audio audio = null;
        private DS4Audio micAudio = null;
        public DateTime lastActive = DateTime.UtcNow;
        public DateTime firstActive = DateTime.UtcNow;
        private bool charging;
        private bool outputRumble = false;
        private int warnInterval = WARN_INTERVAL_USB;
        public int getWarnInterval()
        {
            return warnInterval;
        }

        private bool exitOutputThread = false;
        public bool ExitOutputThread => exitOutputThread;
        private bool exitInputThread = false;
        private object exitLocker = new object();

        public event EventHandler<EventArgs> Report = null;
        public event EventHandler<EventArgs> Removal = null;
        public event EventHandler<EventArgs> SyncChange = null;
        public event EventHandler<EventArgs> SerialChange = null;

        public HidDevice HidDevice => hDevice;
        public bool IsExclusive => HidDevice.IsExclusive;
        public bool isExclusive()
        {
            return HidDevice.IsExclusive;
        }

        private bool isDisconnecting = false;
        public bool IsDisconnecting
        {
            get { return isDisconnecting; }
            private set
            {
                this.isDisconnecting = value;
            }
        }

        public bool isDisconnectingStatus()
        {
            return this.isDisconnecting;
        }

        private bool isRemoving = false;
        public bool IsRemoving
        {
            get { return isRemoving; }
            set
            {
                this.isRemoving = value;
            }
        }

        private bool isRemoved = false;
        public bool IsRemoved
        {
            get { return isRemoved; }
            set
            {
                this.isRemoved = value;
            }
        }

        public object removeLocker = new object();

        public string MacAddress => Mac;
        public string getMacAddress()
        {
            return this.Mac;
        }

        public ConnectionType ConnectionType => conType;
        public ConnectionType getConnectionType()
        {
            return this.conType;
        }

        // behavior only active when > 0
        private int idleTimeout = 0;
        public int IdleTimeout
        {
            get { return idleTimeout; }
            set
            {
                idleTimeout = value;
            }
        }

        public int getIdleTimeout()
        {
            return idleTimeout;
        }

        public void setIdleTimeout(int value)
        {
            if (idleTimeout != value)
            {
                idleTimeout = value;
            }
        }

        public int Battery => battery;
        public int getBattery()
        {
            return battery;
        }

        public bool Charging => charging;
        public bool isCharging()
        {
            return charging;
        }

        private long lastTimeElapsed = 0;
        public long getLastTimeElapsed()
        {
            return lastTimeElapsed;
        }

        public double lastTimeElapsedDouble = 0.0;
        public double getLastTimeElapsedDouble()
        {
            return lastTimeElapsedDouble;
        }

        public byte RightLightFastRumble
        {
            get { return rightLightFastRumble; }
            set
            {
                if (rightLightFastRumble != value)
                    rightLightFastRumble = value;
            }
        }

        public byte LeftHeavySlowRumble
        {
            get { return leftHeavySlowRumble; }
            set
            {
                if (leftHeavySlowRumble != value)
                    leftHeavySlowRumble = value;
            }
        }

        public byte getLeftHeavySlowRumble()
        {
            return leftHeavySlowRumble;
        }

        public DS4Color LightBarColor
        {
            get { return ligtBarColor; }
            set
            {
                if (ligtBarColor.red != value.red || ligtBarColor.green != value.green || ligtBarColor.blue != value.blue)
                {
                    ligtBarColor = value;
                }
            }
        }

        public byte LightBarOnDuration
        {
            get { return ledFlashOn; }
            set
            {
                if (ledFlashOn != value)
                {
                    ledFlashOn = value;
                }
            }
        }

        public byte getLightBarOnDuration()
        {
            return ledFlashOn;
        }

        public byte LightBarOffDuration
        {
            get { return ledFlashOff; }
            set
            {
                if (ledFlashOff != value)
                {
                    ledFlashOff = value;
                }
            }
        }

        public byte getLightBarOffDuration()
        {
            return ledFlashOff;
        }

        // Specify the poll rate interval used for the DS4 hardware when
        // connected via Bluetooth
        private int btPollRate = 0;
        public int BTPollRate
        {
            get { return btPollRate; }
            set
            {
                if (btPollRate != value && value >= 0 && value <= 16)
                {
                    btPollRate = value;
                }
            }
        }

        public int getBTPollRate()
        {
            return btPollRate;
        }

        public void setBTPollRate(int value)
        {
            if (btPollRate != value && value >= 0 && value <= 16)
            {
                btPollRate = value;
            }
        }

        public DS4Touchpad Touchpad { get { return touchpad; } }
        public DS4SixAxis SixAxis { get { return sixAxis; } }

        public static ConnectionType HidConnectionType(HidDevice hidDevice)
        {
            ConnectionType result = ConnectionType.USB;
            if (hidDevice.Capabilities.InputReportByteLength == 64)
            {
                if (hidDevice.Capabilities.NumberFeatureDataIndices == 22)
                {
                    result = ConnectionType.SONYWA;
                }
            }
            else
            {
                result = ConnectionType.BT;
            }

            return result;
        }

        private SynchronizationContext uiContext = null;
        public SynchronizationContext getUiContext()
        {
            return uiContext;
        }
        public void setUiContext(SynchronizationContext uiContext)
        {
            this.uiContext = uiContext;
        }

        private Queue<Action> eventQueue = new Queue<Action>();
        private object eventQueueLock = new object();

        private Thread timeoutCheckThread = null;
        private bool timeoutExecuted = false;
        private bool timeoutEvent = false;

        public DS4Device(HidDevice hidDevice)
        {
            hDevice = hidDevice;
            conType = HidConnectionType(hDevice);
            Mac = hDevice.readSerial();
            if (conType == ConnectionType.USB || conType == ConnectionType.SONYWA)
            {
                inputReport = new byte[64];
                outputReport = new byte[hDevice.Capabilities.OutputReportByteLength];
                outReportBuffer = new byte[hDevice.Capabilities.OutputReportByteLength];
                if (conType == ConnectionType.USB)
                {
                    warnInterval = WARN_INTERVAL_USB;
                    HidDeviceAttributes tempAttr = hDevice.Attributes;
                    if (tempAttr.VendorId == 0x054C && tempAttr.ProductId == 0x09CC)
                    {
                        audio = new DS4Audio();
                        micAudio = new DS4Audio(DS4Library.CoreAudio.DataFlow.Capture);
                    }

                    synced = true;
                }
                else
                {
                    warnInterval = WARN_INTERVAL_BT;
                    audio = new DS4Audio();
                    micAudio = new DS4Audio(DS4Library.CoreAudio.DataFlow.Capture);
                    synced = isValidSerial();
                }
            }
            else
            {
                btInputReport = new byte[BT_INPUT_REPORT_LENGTH];
                inputReport = new byte[BT_INPUT_REPORT_LENGTH - 2];
                outputReport = new byte[BT_OUTPUT_REPORT_LENGTH];
                outReportBuffer = new byte[BT_OUTPUT_REPORT_LENGTH];
                warnInterval = WARN_INTERVAL_BT;
                synced = isValidSerial();
            }

            touchpad = new DS4Touchpad();
            sixAxis = new DS4SixAxis();
            Crc32Algorithm.InitializeTable(DefaultPolynomial);
            refreshCalibration();

            if (!hDevice.IsFileStreamOpen())
            {
                hDevice.OpenFileStream(inputReport.Length);
            }

            sendOutputReport(true, true); // initialize the output report
        }

        private void timeoutTestThread()
        {
            while (!timeoutExecuted)
            {
                if (timeoutEvent)
                {
                    timeoutExecuted = true;
                    this.sendOutputReport(true, true); // Kick Windows into noticing the disconnection.
                }
                else
                {
                    timeoutEvent = true;
                    Thread.Sleep(READ_STREAM_TIMEOUT);
                }
            }
        }

        const int DS4_FEATURE_REPORT_5_LEN = 41;
        const int DS4_FEATURE_REPORT_5_CRC32_POS = DS4_FEATURE_REPORT_5_LEN - 4;
        public void refreshCalibration()
        {
            byte[] calibration = new byte[41];
            calibration[0] = conType == ConnectionType.BT ? (byte)0x05 : (byte)0x02;

            if (conType == ConnectionType.BT)
            {
                bool found = false;
                for (int tries = 0; !found && tries < 5; tries++)
                {
                    hDevice.readFeatureData(calibration);
                    uint recvCrc32 = calibration[DS4_FEATURE_REPORT_5_CRC32_POS] |
                                (uint)(calibration[DS4_FEATURE_REPORT_5_CRC32_POS + 1] << 8) |
                                (uint)(calibration[DS4_FEATURE_REPORT_5_CRC32_POS + 2] << 16) |
                                (uint)(calibration[DS4_FEATURE_REPORT_5_CRC32_POS + 3] << 24);

                    uint calcCrc32 = ~Crc32Algorithm.Compute(new byte[] { 0xA3 });
                    calcCrc32 = ~Crc32Algorithm.CalculateBasicHash(ref calcCrc32, ref calibration, 0, DS4_FEATURE_REPORT_5_LEN - 4);
                    bool validCrc = recvCrc32 == calcCrc32;
                    if (!validCrc && tries >= 5)
                    {
                        Log.LogToGui("Gyro Calibration Failed", true);
                        continue;
                    }
                    else if (validCrc)
                    {
                        found = true;
                    }
                }

                sixAxis.setCalibrationData(ref calibration, conType == ConnectionType.USB);
            }
            else
            {
                hDevice.readFeatureData(calibration);
                sixAxis.setCalibrationData(ref calibration, conType == ConnectionType.USB);
            }
        }

        public void StartUpdate()
        {
            if (ds4Input == null)
            {
                if (conType == ConnectionType.BT)
                {
                    ds4Output = new Thread(performDs4Output);
                    ds4Output.Priority = ThreadPriority.AboveNormal;
                    ds4Output.Name = "DS4 Output thread: " + Mac;
                    ds4Output.IsBackground = true;
                    ds4Output.Start();

                    timeoutCheckThread = new Thread(timeoutTestThread);
                    timeoutCheckThread.Priority = ThreadPriority.BelowNormal;
                    timeoutCheckThread.Name = "DS4 Timeout thread: " + Mac;
                    timeoutCheckThread.IsBackground = true;
                    timeoutCheckThread.Start();
                }

                ds4Input = new Thread(performDs4Input);
                ds4Input.Priority = ThreadPriority.AboveNormal;
                ds4Input.Name = "DS4 Input thread: " + Mac;
                ds4Input.IsBackground = true;
                ds4Input.Start();
            }
            else
                Debug.WriteLine("Thread already running for DS4: " + Mac);
        }

        public void StopUpdate()
        {
            if (ds4Input != null &&
                ds4Input.IsAlive && !ds4Input.ThreadState.HasFlag(System.Threading.ThreadState.Stopped) &&
                !ds4Input.ThreadState.HasFlag(System.Threading.ThreadState.AbortRequested))
            {
                try
                {
                    exitInputThread = true;
                    //ds4Input.Abort();
                    ds4Input.Join();
                }
                catch (Exception e)
                {
                    Debug.WriteLine(e.Message);
                }
            }

            StopOutputUpdate();
        }

        private void StopOutputUpdate()
        {
            lock (exitLocker)
            {
                if (ds4Output != null &&
                    ds4Output.IsAlive && !ds4Output.ThreadState.HasFlag(System.Threading.ThreadState.Stopped) &&
                    !ds4Output.ThreadState.HasFlag(System.Threading.ThreadState.AbortRequested))
                {
                    try
                    {
                        exitOutputThread = true;
                        ds4Output.Interrupt();
                        ds4Output.Join();
                    }
                    catch (Exception e)
                    {
                        Debug.WriteLine(e.Message);
                    }
                }
            }
        }

        private bool writeOutput()
        {
            if (conType == ConnectionType.BT)
            {
                return hDevice.WriteOutputReportViaControl(outputReport);
            }
            else
            {
                return hDevice.WriteOutputReportViaInterrupt(outReportBuffer, READ_STREAM_TIMEOUT);
            }
        }

        private byte outputPendCount = 0;
        private unsafe void performDs4Output()
        {
            try
            {
                int lastError = 0;
                bool result = false, currentRumble = false;
                while (!exitOutputThread)
                {
                    if (currentRumble)
                    {
                        lock (outputReport)
                        {
                            result = writeOutput();
                        }

                        currentRumble = false;
                        if (!result)
                        {
                            currentRumble = true;
                            exitOutputThread = true;
                            int thisError = Marshal.GetLastWin32Error();
                            if (lastError != thisError)
                            {
                                Debug.WriteLine(Mac.ToString() + " " + System.DateTime.UtcNow.ToString("o") + "> encountered write failure: " + thisError);
                                //Log.LogToGui(Mac.ToString() + " encountered write failure: " + thisError, true);
                                lastError = thisError;
                            }
                        }
                    }

                    if (!currentRumble)
                    {
                        lastError = 0;
                        lock (outReportBuffer)
                        {
                            Monitor.Wait(outReportBuffer);
                            fixed (byte* byteR = outputReport, byteB = outReportBuffer)
                            {
                                for (int i = 0, arlen = outputReport.Length; i < arlen; i++)
                                    byteR[i] = byteB[i];
                            }
                            //outReportBuffer.CopyTo(outputReport, 0);
                            outputPendCount--;
                            outputRumble = false;
                        }

                        currentRumble = true;
                    }
                }
            }
            catch (ThreadInterruptedException) { }
        }

        
        public bool IsAlive()
        {
            return priorInputReport30 != 0xff;
        }

        private byte priorInputReport30 = 0xff;

        private bool synced = false;
        public bool Synced
        {
            get { return synced; }
            set
            {
                if (synced != value)
                {
                    synced = value;
                }
            }
        }

        public bool isSynced()
        {
            return synced;
        }

        public double Latency = 0.0;
        public string error;
        public bool firstReport = true;
        public bool oldCharging = false;
        double curTimeDouble = 0.0;
        double oldTimeDouble = 0.0;
        DateTime utcNow = DateTime.UtcNow;
        bool ds4InactiveFrame = true;
        bool idleInput = true;

        bool timeStampInit = false;
        uint timeStampPrevious = 0;
        uint deltaTimeCurrent = 0;


        const int BT_INPUT_REPORT_CRC32_POS = BT_OUTPUT_REPORT_LENGTH - 4; //last 4 bytes of the 78-sized input report are crc32
        const uint DefaultPolynomial = 0xedb88320u;
        uint HamSeed = 2351727372;

        private unsafe void performDs4Input()
        {
            firstActive = DateTime.UtcNow;
            NativeMethods.HidD_SetNumInputBuffers(hDevice.safeReadHandle.DangerousGetHandle(), 2);
            Queue<long> latencyQueue = new Queue<long>(21); // Set capacity at max + 1 to avoid any resizing
            int tempLatencyCount = 0;
            long oldtime = 0;
            string currerror = string.Empty;
            long curtime = 0;
            Stopwatch sw = new Stopwatch();
            sw.Start();
            timeoutEvent = false;
            ds4InactiveFrame = true;
            idleInput = true;
            bool syncWriteReport = conType != ConnectionType.BT;

            int maxBatteryValue = 0;
            int tempBattery = 0;
            uint tempStamp = 0;
            double elapsedDeltaTime = 0.0;
            uint tempDelta = 0;
            byte tempByte = 0;
            int CRC32_POS_1 = BT_INPUT_REPORT_CRC32_POS + 1,
                CRC32_POS_2 = BT_INPUT_REPORT_CRC32_POS + 2,
                CRC32_POS_3 = BT_INPUT_REPORT_CRC32_POS + 3;
            int crcpos = BT_INPUT_REPORT_CRC32_POS;
            int crcoffset = 0;

            while (!exitInputThread)
            {
                oldCharging = charging;
                currerror = string.Empty;

                if (tempLatencyCount >= 20)
                {
                    latencyQueue.Dequeue();
                    tempLatencyCount--;
                }

                latencyQueue.Enqueue(this.lastTimeElapsed);
                tempLatencyCount++;

                Latency = latencyQueue.Average();

                if (conType == ConnectionType.BT)
                {
                    //HidDevice.ReadStatus res = hDevice.ReadFile(btInputReport);
                    //HidDevice.ReadStatus res = hDevice.ReadAsyncWithFileStream(btInputReport, READ_STREAM_TIMEOUT);
                    HidDevice.ReadStatus res = hDevice.ReadWithFileStream(btInputReport);
                    timeoutEvent = false;
                    if (res == HidDevice.ReadStatus.Success)
                    {
                        //Array.Copy(btInputReport, 2, inputReport, 0, inputReport.Length);
                        fixed (byte* byteP = &btInputReport[2], imp = inputReport)
                        {
                            for (int j = 0; j < BT_INPUT_REPORT_LENGTH - 2; j++)
                            {
                                imp[j] = byteP[j];
                            }
                        }

                        //uint recvCrc32 = BitConverter.ToUInt32(btInputReport, BT_INPUT_REPORT_CRC32_POS);
                        uint recvCrc32 = btInputReport[BT_INPUT_REPORT_CRC32_POS] |
                            (uint)(btInputReport[CRC32_POS_1] << 8) |
                            (uint)(btInputReport[CRC32_POS_2] << 16) |
                            (uint)(btInputReport[CRC32_POS_3] << 24);

                        uint calcCrc32 = ~Crc32Algorithm.CalculateFasterBTHash(ref HamSeed, ref btInputReport, ref crcoffset, ref crcpos);
                        if (recvCrc32 != calcCrc32)
                        {
                            //Log.LogToGui("Crc check failed", true);
                            //Debug.WriteLine(MacAddress.ToString() + " " + System.DateTime.UtcNow.ToString("o") + "" +
                            //                    "> invalid CRC32 in BT input report: 0x" + recvCrc32.ToString("X8") + " expected: 0x" + calcCrc32.ToString("X8"));

                            //cState.PacketCounter = pState.PacketCounter + 1; //still increase so we know there were lost packets
                            continue;
                        }
                    }
                    else
                    {
                        if (res == HidDevice.ReadStatus.WaitTimedOut)
                        {
                            Log.LogToGui(Mac.ToString() + " disconnected due to timeout", true);
                        }
                        else
                        {
                            int winError = Marshal.GetLastWin32Error();
                            Debug.WriteLine(Mac.ToString() + " " + DateTime.UtcNow.ToString("o") + "> disconnect due to read failure: " + winError);
                            //Log.LogToGui(Mac.ToString() + " disconnected due to read failure: " + winError, true);
                        }

                        sendOutputReport(true, true); // Kick Windows into noticing the disconnection.
                        StopOutputUpdate();
                        isDisconnecting = true;
                        uiContext.Send(new SendOrPostCallback(delegate (object state4)
                        {
                            Removal?.Invoke(this, EventArgs.Empty);
                        }), null);

                        timeoutExecuted = true;
                        return;
                    }
                }
                else
                {
                    //HidDevice.ReadStatus res = hDevice.ReadFile(inputReport);
                    //Array.Clear(inputReport, 0, inputReport.Length);
                    //HidDevice.ReadStatus res = hDevice.ReadAsyncWithFileStream(inputReport, READ_STREAM_TIMEOUT);
                    HidDevice.ReadStatus res = hDevice.ReadWithFileStream(inputReport);
                    if (res != HidDevice.ReadStatus.Success)
                    {
                        if (res == HidDevice.ReadStatus.WaitTimedOut)
                        {
                            Log.LogToGui(Mac.ToString() + " disconnected due to timeout", true);
                        }
                        else
                        {
                            int winError = Marshal.GetLastWin32Error();
                            Debug.WriteLine(Mac.ToString() + " " + DateTime.UtcNow.ToString("o") + "> disconnect due to read failure: " + winError);
                            //Log.LogToGui(Mac.ToString() + " disconnected due to read failure: " + winError, true);
                        }

                        StopOutputUpdate();
                        isDisconnecting = true;
                        uiContext.Send(new SendOrPostCallback(delegate (object state4)
                        {
                            Removal?.Invoke(this, EventArgs.Empty);
                        }), null);

                        timeoutExecuted = true;
                        return;
                    }
                }

                curTimeDouble = sw.Elapsed.TotalMilliseconds;
                curtime = sw.ElapsedMilliseconds;

                lastTimeElapsed = curtime - oldtime;
                lastTimeElapsedDouble = (curTimeDouble - oldTimeDouble);

                oldtime = curtime;
                oldTimeDouble = curTimeDouble;

                if (conType == ConnectionType.BT && btInputReport[0] != 0x11)
                {
                    //Received incorrect report, skip it
                    continue;
                }

                utcNow = DateTime.UtcNow; // timestamp with UTC in case system time zone changes
                resetHapticState();
                cState.ReportTimeStamp = utcNow;
                cState.LX = inputReport[1];
                cState.LY = inputReport[2];
                cState.RX = inputReport[3];
                cState.RY = inputReport[4];
                cState.L2 = inputReport[8];
                cState.R2 = inputReport[9];

                tempByte = inputReport[5];
                cState.Triangle = (tempByte & (1 << 7)) != 0;
                cState.Circle = (tempByte & (1 << 6)) != 0;
                cState.Cross = (tempByte & (1 << 5)) != 0;
                cState.Square = (tempByte & (1 << 4)) != 0;

                // First 4 bits denote dpad state. Clock representation
                // with 8 meaning centered and 0 meaning DpadUp.
                byte dpad_state = (byte)(tempByte & 0x0F);

                switch (dpad_state)
                {
                    case 0: cState.DpadUp = true; cState.DpadDown = false; cState.DpadLeft = false; cState.DpadRight = false; break;
                    case 1: cState.DpadUp = true; cState.DpadDown = false; cState.DpadLeft = false; cState.DpadRight = true; break;
                    case 2: cState.DpadUp = false; cState.DpadDown = false; cState.DpadLeft = false; cState.DpadRight = true; break;
                    case 3: cState.DpadUp = false; cState.DpadDown = true; cState.DpadLeft = false; cState.DpadRight = true; break;
                    case 4: cState.DpadUp = false; cState.DpadDown = true; cState.DpadLeft = false; cState.DpadRight = false; break;
                    case 5: cState.DpadUp = false; cState.DpadDown = true; cState.DpadLeft = true; cState.DpadRight = false; break;
                    case 6: cState.DpadUp = false; cState.DpadDown = false; cState.DpadLeft = true; cState.DpadRight = false; break;
                    case 7: cState.DpadUp = true; cState.DpadDown = false; cState.DpadLeft = true; cState.DpadRight = false; break;
                    case 8:
                    default: cState.DpadUp = false; cState.DpadDown = false; cState.DpadLeft = false; cState.DpadRight = false; break;
                }

                tempByte = inputReport[6];
                cState.R3 = (tempByte & (1 << 7)) != 0;
                cState.L3 = (tempByte & (1 << 6)) != 0;
                cState.Options = (tempByte & (1 << 5)) != 0;
                cState.Share = (tempByte & (1 << 4)) != 0;
                cState.R1 = (tempByte & (1 << 1)) != 0;
                cState.L1 = (tempByte & (1 << 0)) != 0;

                tempByte = inputReport[7];
                cState.PS = (tempByte & (1 << 0)) != 0;
                cState.TouchButton = (tempByte & 0x02) != 0;
                cState.FrameCounter = (byte)(tempByte >> 2);

                tempByte = inputReport[30];
                charging = (tempByte & 0x10) != 0;
                maxBatteryValue = charging ? BATTERY_MAX_USB : BATTERY_MAX;
                tempBattery = (tempByte & 0x0f) * 100 / maxBatteryValue;
                battery = Math.Min((byte)tempBattery, (byte)100);
                cState.Battery = (byte)battery;
                //System.Diagnostics.Debug.WriteLine("CURRENT BATTERY: " + (inputReport[30] & 0x0f) + " | " + tempBattery + " | " + battery);
                if (tempByte != priorInputReport30)
                {
                    priorInputReport30 = tempByte;
                    //Debug.WriteLine(MacAddress.ToString() + " " + System.DateTime.UtcNow.ToString("o") + "> power subsystem octet: 0x" + inputReport[30].ToString("x02"));
                }

                tempStamp = (uint)((ushort)(inputReport[11] << 8) | inputReport[10]);
                if (timeStampInit == false)
                {
                    timeStampInit = true;
                    deltaTimeCurrent = tempStamp * 16u / 3u;
                }
                else if (timeStampPrevious > tempStamp)
                {
                    tempDelta = ushort.MaxValue - timeStampPrevious + tempStamp + 1u;
                    deltaTimeCurrent = tempDelta * 16u / 3u;
                }
                else
                {
                    tempDelta = tempStamp - timeStampPrevious;
                    deltaTimeCurrent = tempDelta * 16u / 3u;
                }

                timeStampPrevious = tempStamp;
                elapsedDeltaTime = 0.000001 * deltaTimeCurrent; // Convert from microseconds to seconds
                cState.elapsedTime = elapsedDeltaTime;

                // XXX DS4State mapping needs fixup, turn touches into an array[4] of structs.  And include the touchpad details there instead.
                try
                {
                    // Only care if one touch packet is detected. Other touch packets
                    // don't seem to contain relevant data. ds4drv does not use them either.
                    for (int touches = Math.Max((int)(inputReport[-1 + DS4Touchpad.TOUCHPAD_DATA_OFFSET - 1]), 1), touchOffset = 0; touches > 0; touches--, touchOffset += 9)
                    //for (int touches = inputReport[-1 + DS4Touchpad.TOUCHPAD_DATA_OFFSET - 1], touchOffset = 0; touches > 0; touches--, touchOffset += 9)
                    {
                        cState.TouchPacketCounter = inputReport[-1 + DS4Touchpad.TOUCHPAD_DATA_OFFSET + touchOffset];
                        cState.Touch1 = (inputReport[0 + DS4Touchpad.TOUCHPAD_DATA_OFFSET + touchOffset] >> 7) != 0 ? false : true; // finger 1 detected
                        cState.Touch1Identifier = (byte)(inputReport[0 + DS4Touchpad.TOUCHPAD_DATA_OFFSET + touchOffset] & 0x7f);
                        cState.Touch2 = (inputReport[4 + DS4Touchpad.TOUCHPAD_DATA_OFFSET + touchOffset] >> 7) != 0 ? false : true; // finger 2 detected
                        cState.Touch2Identifier = (byte)(inputReport[4 + DS4Touchpad.TOUCHPAD_DATA_OFFSET + touchOffset] & 0x7f);
                        cState.Touch1Finger = cState.Touch1 || cState.Touch2; // >= 1 touch detected
                        cState.Touch2Fingers = cState.Touch1 && cState.Touch2; // 2 touches detected
                        int touchX = (((inputReport[2 + DS4Touchpad.TOUCHPAD_DATA_OFFSET + touchOffset] & 0xF) << 8) | inputReport[1 + DS4Touchpad.TOUCHPAD_DATA_OFFSET + touchOffset]);
                        cState.TouchLeft = touchX >= 1920 * 2 / 5 ? false : true;
                        cState.TouchRight = touchX < 1920 * 2 / 5 ? false : true;
                        // Even when idling there is still a touch packet indicating no touch 1 or 2
                        touchpad.handleTouchpad(inputReport, cState, touchOffset);
                    }
                }
                catch { currerror = "Index out of bounds: touchpad"; }

                // Store Gyro and Accel values
                //Array.Copy(inputReport, 13, gyro, 0, 6);
                //Array.Copy(inputReport, 19, accel, 0, 6);
                fixed (byte* pbInput = &inputReport[13], pbGyro = gyro, pbAccel = accel)
                {
                    for (int i = 0; i < 6; i++)
                    {
                        pbGyro[i] = pbInput[i];
                    }

                    for (int i = 6; i < 12; i++)
                    {
                        pbAccel[i - 6] = pbInput[i];
                    }
                }
                sixAxis.handleSixaxis(gyro, accel, cState, elapsedDeltaTime);

                if (conType == ConnectionType.SONYWA)
                {
                    bool controllerSynced = inputReport[31] == 0;
                    if (controllerSynced != synced)
                    {
                        synced = controllerSynced;
                        SyncChange?.Invoke(this, EventArgs.Empty);
                        sendOutputReport(true, true);
                    }
                }

                ds4InactiveFrame = cState.FrameCounter == pState.FrameCounter;
                if (!ds4InactiveFrame)
                {
                    isRemoved = false;
                }

                if (conType == ConnectionType.USB)
                {
                    if (idleTimeout == 0)
                    {
                        lastActive = utcNow;
                    }
                    else
                    {
                        idleInput = isDS4Idle();
                        if (!idleInput)
                        {
                            lastActive = utcNow;
                        }
                    }
                }
                else
                {
                    bool shouldDisconnect = false;
                    if (!isRemoved && idleTimeout > 0)
                    {
                        idleInput = isDS4Idle();
                        if (idleInput)
                        {
                            DateTime timeout = lastActive + TimeSpan.FromSeconds(idleTimeout);
                            if (!charging)
                                shouldDisconnect = utcNow >= timeout;
                        }
                        else
                        {
                            lastActive = utcNow;
                        }
                    }
                    else
                    {
                        lastActive = utcNow;
                    }

                    if (shouldDisconnect)
                    {
                        Log.LogToGui(Mac.ToString() + " disconnecting due to idle disconnect", false);

                        if (conType == ConnectionType.BT)
                        {
                            if (DisconnectBT(true))
                            {
                                timeoutExecuted = true;
                                return; // all done
                            }
                        }
                        else if (conType == ConnectionType.SONYWA)
                        {
                            DisconnectDongle();
                        }
                    }
                }

                if (conType == ConnectionType.BT && oldCharging != charging)
                {
                    if (Global.getQuickCharge() && charging)
                    {
                        DisconnectBT(true);
                        timeoutExecuted = true;
                        return;
                    }
                }

                if (Report != null)
                    Report(this, EventArgs.Empty);

                sendOutputReport(syncWriteReport);

                if (!string.IsNullOrEmpty(currerror))
                    error = currerror;
                else if (!string.IsNullOrEmpty(error))
                    error = string.Empty;

                cState.CopyTo(pState);

                lock (eventQueueLock)
                {
                    Action tempAct = null;
                    for (int actInd = 0, actLen = eventQueue.Count; actInd < actLen; actInd++)
                    {
                        tempAct = eventQueue.Dequeue();
                        tempAct.Invoke();
                    }
                }
            }

            timeoutExecuted = true;
        }

        public void FlushHID()
        {
            hDevice.flush_Queue();
        }

        private unsafe void sendOutputReport(bool synchronous, bool force = false)
        {
            setTestRumble();
            setHapticState();

            bool quitOutputThread = false;
            bool usingBT = conType == ConnectionType.BT;

            lock (outReportBuffer)
            {
                bool output = outputPendCount > 0, change = force;

                if (usingBT)
                {
                    outReportBuffer[0] = 0x11;
                    outReportBuffer[1] = (byte)(0x80 | btPollRate); // input report rate
                    // enable rumble (0x01), lightbar (0x02), flash (0x04)
                    outReportBuffer[3] = 0xf7;
                    outReportBuffer[6] = rightLightFastRumble; // fast motor
                    outReportBuffer[7] = leftHeavySlowRumble; // slow motor
                    outReportBuffer[8] = ligtBarColor.red; // red
                    outReportBuffer[9] = ligtBarColor.green; // green
                    outReportBuffer[10] = ligtBarColor.blue; // blue
                    outReportBuffer[11] = ledFlashOn; // flash on duration
                    outReportBuffer[12] = ledFlashOff; // flash off duration

                    fixed (byte* byteR = outputReport, byteB = outReportBuffer)
                    {
                        for (int i = 0, arlen = 13; !change && i < arlen; i++)
                            change = byteR[i] != byteB[i];
                    }
                }
                else
                {
                    outReportBuffer[0] = 0x05;
                    // enable rumble (0x01), lightbar (0x02), flash (0x04)
                    outReportBuffer[1] = 0xf7;
                    outReportBuffer[4] = rightLightFastRumble; // fast motor
                    outReportBuffer[5] = leftHeavySlowRumble; // slow  motor
                    outReportBuffer[6] = ligtBarColor.red; // red
                    outReportBuffer[7] = ligtBarColor.green; // green
                    outReportBuffer[8] = ligtBarColor.blue; // blue
                    outReportBuffer[9] = ledFlashOn; // flash on duration
                    outReportBuffer[10] = ledFlashOff; // flash off duration

                    fixed (byte* byteR = outputReport, byteB = outReportBuffer)
                    {
                        for (int i = 0, arlen = 11; !change && i < arlen; i++)
                            change = byteR[i] != byteB[i];
                    }

                    if (change && audio != null)
                    {
                        // Headphone volume levels
                        outReportBuffer[19] = outReportBuffer[20] =
                            Convert.ToByte(audio.getVolume());
                        // Microphone volume level
                        outReportBuffer[21] = Convert.ToByte(micAudio.getVolume());
                    }
                }

                if (synchronous)
                {
                    outputRumble = false;
                    outputPendCount = 3;

                    if (change)
                    {
                        if (usingBT)
                        {
                            Monitor.Enter(outputReport);
                            outReportBuffer.CopyTo(outputReport, 0);
                        }

                        try
                        {
                            if (!writeOutput())
                            {
                                int winError = Marshal.GetLastWin32Error();
                                quitOutputThread = true;
                            }
                        }
                        catch { } // If it's dead already, don't worry about it.

                        if (usingBT)
                        {
                            Monitor.Exit(outputReport);
                        }
                    }
                }
                else
                {
                    //for (int i = 0, arlen = outputReport.Length; !change && i < arlen; i++)
                    //    change = outputReport[i] != outReportBuffer[i];

                    if (output || change)
                    {
                        if (change)
                        {
                            outputPendCount = 3;
                        }

                        outputRumble = true;
                        Monitor.Pulse(outReportBuffer);
                    }
                }
            }

            if (quitOutputThread)
            {
                StopOutputUpdate();
                exitOutputThread = true;
            }
        }

        public bool DisconnectBT(bool callRemoval = false)
        {
            if (Mac != null)
            {
                // Wait for output report to be written
                StopOutputUpdate();
                Debug.WriteLine("Trying to disconnect BT device " + Mac);
                IntPtr btHandle = IntPtr.Zero;
                int IOCTL_BTH_DISCONNECT_DEVICE = 0x41000c;

                byte[] btAddr = new byte[8];
                string[] sbytes = Mac.Split(':');
                for (int i = 0; i < 6; i++)
                {
                    // parse hex byte in reverse order
                    btAddr[5 - i] = Convert.ToByte(sbytes[i], 16);
                }

                long lbtAddr = BitConverter.ToInt64(btAddr, 0);

                bool success = false;

                lock (outputReport)
                {
                    NativeMethods.BLUETOOTH_FIND_RADIO_PARAMS p = new NativeMethods.BLUETOOTH_FIND_RADIO_PARAMS();
                    p.dwSize = Marshal.SizeOf(typeof(NativeMethods.BLUETOOTH_FIND_RADIO_PARAMS));
                    IntPtr searchHandle = NativeMethods.BluetoothFindFirstRadio(ref p, ref btHandle);
                    int bytesReturned = 0;

                    while (!success && btHandle != IntPtr.Zero)
                    {
                        success = NativeMethods.DeviceIoControl(btHandle, IOCTL_BTH_DISCONNECT_DEVICE, ref lbtAddr, 8, IntPtr.Zero, 0, ref bytesReturned, IntPtr.Zero);
                        NativeMethods.CloseHandle(btHandle);
                        if (!success)
                        {
                            if (!NativeMethods.BluetoothFindNextRadio(searchHandle, ref btHandle))
                                btHandle = IntPtr.Zero;
                        }
                    }

                    NativeMethods.BluetoothFindRadioClose(searchHandle);
                    Debug.WriteLine("Disconnect successful: " + success);
                }

                success = true; // XXX return value indicates failure, but it still works?
                if (success)
                {
                    IsDisconnecting = true;

                    if (callRemoval)
                    {
                        uiContext.Send(new SendOrPostCallback(delegate (object state)
                        {
                            Removal?.Invoke(this, EventArgs.Empty);
                        }), null);

                        //System.Threading.Tasks.Task.Factory.StartNew(() => { Removal?.Invoke(this, EventArgs.Empty); });
                    }
                }

                return success;
            }

            return false;
        }

        public bool DisconnectDongle(bool remove = false)
        {
            bool result = false;
            byte[] disconnectReport = new byte[65];
            disconnectReport[0] = 0xe2;
            disconnectReport[1] = 0x02;
            Array.Clear(disconnectReport, 2, 63);

            if (remove)
                StopOutputUpdate();

            lock (outputReport)
            {
                result = hDevice.WriteFeatureReport(disconnectReport);
            }

            if (result && remove)
            {
                isDisconnecting = true;

                uiContext.Send(new SendOrPostCallback(delegate (object state4)
                {
                    Removal?.Invoke(this, EventArgs.Empty);
                }), null);

                //System.Threading.Tasks.Task.Factory.StartNew(() => { Removal?.Invoke(this, EventArgs.Empty); });
                //Removal?.Invoke(this, EventArgs.Empty);
            }
            else if (result && !remove)
            {
                isRemoved = true;
            }

            return result;
        }

        private DS4HapticState testRumble = new DS4HapticState();

        public void setRumble(byte rightLightFastMotor, byte leftHeavySlowMotor)
        {
            testRumble.RumbleMotorStrengthRightLightFast = rightLightFastMotor;
            testRumble.RumbleMotorStrengthLeftHeavySlow = leftHeavySlowMotor;
            testRumble.RumbleMotorsExplicitlyOff = rightLightFastMotor == 0 && leftHeavySlowMotor == 0;
        }

        private void setTestRumble()
        {
            if (testRumble.IsRumbleSet())
            {
                pushHapticState(ref testRumble);
                if (testRumble.RumbleMotorsExplicitlyOff)
                    testRumble.RumbleMotorsExplicitlyOff = false;
            }
        }

        public DS4State getCurrentState()
        {
            return cState.Clone();
        }

        public DS4State getPreviousState()
        {
            return pState.Clone();
        }

        public void getCurrentState(DS4State state)
        {
            cState.CopyTo(state);
        }

        public void getPreviousState(DS4State state)
        {
            pState.CopyTo(state);
        }

        public DS4State getCurrentStateRef()
        {
            return cState;
        }

        public DS4State getPreviousStateRef()
        {
            return pState;
        }

        private bool isDS4Idle()
        {
            if (cState.Square || cState.Cross || cState.Circle || cState.Triangle)
                return false;
            if (cState.DpadUp || cState.DpadLeft || cState.DpadDown || cState.DpadRight)
                return false;
            if (cState.L3 || cState.R3 || cState.L1 || cState.R1 || cState.Share || cState.Options)
                return false;
            if (cState.L2 != 0 || cState.R2 != 0)
                return false;
            // TODO calibrate to get an accurate jitter and center-play range and centered position
            const int slop = 64;
            if (cState.LX <= 127 - slop || cState.LX >= 128 + slop || cState.LY <= 127 - slop || cState.LY >= 128 + slop)
                return false;
            if (cState.RX <= 127 - slop || cState.RX >= 128 + slop || cState.RY <= 127 - slop || cState.RY >= 128 + slop)
                return false;
            if (cState.Touch1 || cState.Touch2 || cState.TouchButton)
                return false;
            return true;
        }

        private DS4HapticState[] hapticState = new DS4HapticState[1];
        private int hapticStackIndex = 0;
        private void resetHapticState()
        {
            hapticStackIndex = 0;
        }

        delegate void HapticItem(ref DS4HapticState haptic);

        // Use the "most recently set" haptic state for each of light bar/motor.
        private void setHapticState()
        {
            byte lightBarFlashDurationOn = ledFlashOn, lightBarFlashDurationOff = ledFlashOff;
            byte rumbleMotorStrengthLeftHeavySlow = leftHeavySlowRumble,
                rumbleMotorStrengthRightLightFast = rightLightFastRumble;
            int hapticLen = hapticState.Length;
            for (int i = 0; i < hapticLen; i++)
            {
                if (i == hapticStackIndex)
                    break; // rest haven't been used this time

                ((HapticItem)((ref DS4HapticState haptic) => {
                    if (haptic.IsLightBarSet())
                    {
                        ligtBarColor = haptic.LightBarColor;
                        lightBarFlashDurationOn = haptic.LightBarFlashDurationOn;
                        lightBarFlashDurationOff = haptic.LightBarFlashDurationOff;
                    }

                    if (haptic.IsRumbleSet())
                    {
                        rumbleMotorStrengthLeftHeavySlow = haptic.RumbleMotorStrengthLeftHeavySlow;
                        rumbleMotorStrengthRightLightFast = haptic.RumbleMotorStrengthRightLightFast;
                    }
                }))(ref hapticState[i]);
            }

            ledFlashOn = lightBarFlashDurationOn;
            ledFlashOff = lightBarFlashDurationOff;
            leftHeavySlowRumble = rumbleMotorStrengthLeftHeavySlow;
            rightLightFastRumble = rumbleMotorStrengthRightLightFast;
        }

        public void pushHapticState(ref DS4HapticState hs)
        {
            int hapsLen = hapticState.Length;
            if (hapticStackIndex == hapsLen)
            {
                DS4HapticState[] newHaptics = new DS4HapticState[hapsLen + 1];
                Array.Copy(hapticState, newHaptics, hapsLen);
                hapticState = newHaptics;
            }

            hapticState[hapticStackIndex++] = hs;
        }

        override
        public string ToString()
        {
            return Mac;
        }

        public void runRemoval()
        {
            Removal?.Invoke(this, EventArgs.Empty);
        }

        public void removeReportHandlers()
        {
            this.Report = null;
        }

        public void queueEvent(Action act)
        {
            lock (eventQueueLock)
            {
                eventQueue.Enqueue(act);
            }
        }

        public void updateSerial()
        {
            hDevice.resetSerial();
            string tempMac = hDevice.readSerial();
            if (tempMac != Mac)
            {
                Mac = tempMac;
                SerialChange?.Invoke(this, EventArgs.Empty);
            }
        }

        public bool isValidSerial()
        {
            return !Mac.Equals(blankSerial);
        }

        public static bool isValidSerial(string test)
        {
            return !test.Equals(blankSerial);
        }
    }
    //DS4DEVICES
    public class VidPidInfo
    {
        public readonly int vid;
        public readonly int pid;
        internal VidPidInfo(int vid, int pid)
        {
            this.vid = vid;
            this.pid = pid;
        }
    }
    public class DS4Devices
    {
        // (HID device path, DS4Device)
        private static Dictionary<string, DS4Device> Devices = new Dictionary<string, DS4Device>();
        private static HashSet<string> deviceSerials = new HashSet<string>();
        private static HashSet<string> DevicePaths = new HashSet<string>();
        // Keep instance of opened exclusive mode devices not in use (Charging while using BT connection)
        private static List<HidDevice> DisabledDevices = new List<HidDevice>();
        private static Stopwatch sw = new Stopwatch();
        public static bool isExclusiveMode = false;
        internal const int SONY_VID = 0x054C;
        internal const int RAZER_VID = 0x1532;
        internal const int NACON_VID = 0x146B;
        internal const int HORI_VID = 0x0F0D;

        private static VidPidInfo[] knownDevices =
        {
            new VidPidInfo(SONY_VID, 0xBA0),
            new VidPidInfo(SONY_VID, 0x5C4),
            new VidPidInfo(SONY_VID, 0x09CC),
            new VidPidInfo(RAZER_VID, 0x1000),
            new VidPidInfo(NACON_VID, 0x0D01),
            new VidPidInfo(HORI_VID, 0x00EE)    // Hori PS4 Mini Wired Gamepad
        };

        private static string devicePathToInstanceId(string devicePath)
        {
            string deviceInstanceId = devicePath;
            deviceInstanceId = deviceInstanceId.Remove(0, deviceInstanceId.LastIndexOf('\\') + 1);
            deviceInstanceId = deviceInstanceId.Remove(deviceInstanceId.LastIndexOf('{'));
            deviceInstanceId = deviceInstanceId.Replace('#', '\\');
            if (deviceInstanceId.EndsWith("\\"))
            {
                deviceInstanceId = deviceInstanceId.Remove(deviceInstanceId.Length - 1);
            }

            return deviceInstanceId;
        }

        // Enumerates ds4 controllers in the system
        public static void findControllers()
        {
            lock (Devices)
            {
                IEnumerable<HidDevice> hDevices = HidDevices.EnumerateDS4(knownDevices);
                // Sort Bluetooth first in case USB is also connected on the same controller.
                hDevices = hDevices.OrderBy<HidDevice, ConnectionType>((HidDevice d) => { return DS4Device.HidConnectionType(d); });

                List<HidDevice> tempList = hDevices.ToList();
                purgeHiddenExclusiveDevices();
                tempList.AddRange(DisabledDevices);
                int devCount = tempList.Count();
                string devicePlural = "device" + (devCount == 0 || devCount > 1 ? "s" : "");
                //Log.LogToGui("Found " + devCount + " possible " + devicePlural + ". Examining " + devicePlural + ".", false);

                for (int i = 0; i < devCount; i++)
                //foreach (HidDevice hDevice in hDevices)
                {
                    HidDevice hDevice = tempList[i];
                    if (hDevice.Description == "HID-compliant vendor-defined device")
                        continue; // ignore the Nacon Revolution Pro programming interface
                    else if (DevicePaths.Contains(hDevice.DevicePath))
                        continue; // BT/USB endpoint already open once

                    if (!hDevice.IsOpen)
                    {
                        hDevice.OpenDevice(isExclusiveMode);
                        if (!hDevice.IsOpen && isExclusiveMode)
                        {
                            try
                            {
                                WindowsIdentity identity = WindowsIdentity.GetCurrent();
                                WindowsPrincipal principal = new WindowsPrincipal(identity);
                                bool elevated = principal.IsInRole(WindowsBuiltInRole.Administrator);

                                if (!elevated)
                                {
                                    // Launches an elevated child process to re-enable device
                                    string exeName = Process.GetCurrentProcess().MainModule.FileName;
                                    ProcessStartInfo startInfo = new ProcessStartInfo(exeName);
                                    startInfo.Verb = "runas";
                                    startInfo.Arguments = "re-enabledevice " + devicePathToInstanceId(hDevice.DevicePath);
                                    Process child = Process.Start(startInfo);

                                    if (!child.WaitForExit(5000))
                                    {
                                        child.Kill();
                                    }
                                    else if (child.ExitCode == 0)
                                    {
                                        hDevice.OpenDevice(isExclusiveMode);
                                    }
                                }
                                else
                                {
                                    reEnableDevice(devicePathToInstanceId(hDevice.DevicePath));
                                    hDevice.OpenDevice(isExclusiveMode);
                                }
                            }
                            catch (Exception) { }
                        }

                        // TODO in exclusive mode, try to hold both open when both are connected
                        if (isExclusiveMode && !hDevice.IsOpen)
                            hDevice.OpenDevice(false);
                    }

                    if (hDevice.IsOpen)
                    {
                        string serial = hDevice.readSerial();
                        bool validSerial = !serial.Equals(DS4Device.blankSerial);
                        if (validSerial && deviceSerials.Contains(serial))
                        {
                            // happens when the BT endpoint already is open and the USB is plugged into the same host
                            if (isExclusiveMode && hDevice.IsExclusive &&
                                !DisabledDevices.Contains(hDevice))
                            {
                                // Grab reference to exclusively opened HidDevice so device
                                // stays hidden to other processes
                                DisabledDevices.Add(hDevice);
                                //DevicePaths.Add(hDevice.DevicePath);
                            }

                            continue;
                        }
                        else
                        {
                            DS4Device ds4Device = new DS4Device(hDevice);
                            //ds4Device.Removal += On_Removal;
                            if (!ds4Device.ExitOutputThread)
                            {
                                Devices.Add(hDevice.DevicePath, ds4Device);
                                DevicePaths.Add(hDevice.DevicePath);
                                deviceSerials.Add(serial);
                            }
                        }
                    }
                }
            }
        }

        // Returns DS4 controllers that were found and are running
        public static IEnumerable<DS4Device> getDS4Controllers()
        {
            lock (Devices)
            {
                DS4Device[] controllers = new DS4Device[Devices.Count];
                Devices.Values.CopyTo(controllers, 0);
                return controllers;
            }
        }

        public static void stopControllers()
        {
            lock (Devices)
            {
                IEnumerable<DS4Device> devices = getDS4Controllers();
                //foreach (DS4Device device in devices)
                for (int i = 0, devCount = devices.Count(); i < devCount; i++)
                {
                    DS4Device device = devices.ElementAt(i);
                    device.StopUpdate();
                    //device.runRemoval();
                    device.HidDevice.CloseDevice();
                }

                Devices.Clear();
                DevicePaths.Clear();
                deviceSerials.Clear();
                DisabledDevices.Clear();
            }
        }

        // Called when devices is diconnected, timed out or has input reading failure
        public static void On_Removal(object sender, EventArgs e)
        {
            DS4Device device = (DS4Device)sender;
            RemoveDevice(device);
        }

        public static void RemoveDevice(DS4Device device)
        {
            lock (Devices)
            {
                if (device != null)
                {
                    device.HidDevice.CloseDevice();
                    Devices.Remove(device.HidDevice.DevicePath);
                    DevicePaths.Remove(device.HidDevice.DevicePath);
                    deviceSerials.Remove(device.MacAddress);
                    //purgeHiddenExclusiveDevices();
                }
            }
        }

        public static void UpdateSerial(object sender, EventArgs e)
        {
            lock (Devices)
            {
                DS4Device device = (DS4Device)sender;
                if (device != null)
                {
                    string devPath = device.HidDevice.DevicePath;
                    string serial = device.getMacAddress();
                    if (Devices.ContainsKey(devPath))
                    {
                        deviceSerials.Remove(serial);
                        device.updateSerial();
                        serial = device.getMacAddress();
                        if (DS4Device.isValidSerial(serial))
                        {
                            deviceSerials.Add(serial);
                        }

                        device.refreshCalibration();
                    }
                }
            }
        }

        private static void purgeHiddenExclusiveDevices()
        {
            int disabledDevCount = DisabledDevices.Count;
            if (disabledDevCount > 0)
            {
                List<HidDevice> disabledDevList = new List<HidDevice>();
                for (int i = 0, arlen = disabledDevCount; i < arlen; i++)
                {
                    HidDevice tempDev = DisabledDevices.ElementAt(i);
                    if (tempDev != null)
                    {
                        if (tempDev.IsOpen && tempDev.IsConnected)
                        {
                            disabledDevList.Add(tempDev);
                        }
                        else if (tempDev.IsOpen)
                        {
                            if (!tempDev.IsConnected)
                            {
                                try
                                {
                                    tempDev.CloseDevice();
                                }
                                catch { }
                            }

                            if (DevicePaths.Contains(tempDev.DevicePath))
                            {
                                DevicePaths.Remove(tempDev.DevicePath);
                            }
                        }
                    }
                }

                DisabledDevices.Clear();
                DisabledDevices.AddRange(disabledDevList);
            }
        }

        public static void reEnableDevice(string deviceInstanceId)
        {
            bool success;
            Guid hidGuid = new Guid();
            NativeMethods.HidD_GetHidGuid(ref hidGuid);
            IntPtr deviceInfoSet = NativeMethods.SetupDiGetClassDevs(ref hidGuid, deviceInstanceId, 0, NativeMethods.DIGCF_PRESENT | NativeMethods.DIGCF_DEVICEINTERFACE);
            NativeMethods.SP_DEVINFO_DATA deviceInfoData = new NativeMethods.SP_DEVINFO_DATA();
            deviceInfoData.cbSize = Marshal.SizeOf(deviceInfoData);
            success = NativeMethods.SetupDiEnumDeviceInfo(deviceInfoSet, 0, ref deviceInfoData);
            if (!success)
            {
                throw new Exception("Error getting device info data, error code = " + Marshal.GetLastWin32Error());
            }
            success = NativeMethods.SetupDiEnumDeviceInfo(deviceInfoSet, 1, ref deviceInfoData); // Checks that we have a unique device
            if (success)
            {
                throw new Exception("Can't find unique device");
            }

            NativeMethods.SP_PROPCHANGE_PARAMS propChangeParams = new NativeMethods.SP_PROPCHANGE_PARAMS();
            propChangeParams.classInstallHeader.cbSize = Marshal.SizeOf(propChangeParams.classInstallHeader);
            propChangeParams.classInstallHeader.installFunction = NativeMethods.DIF_PROPERTYCHANGE;
            propChangeParams.stateChange = NativeMethods.DICS_DISABLE;
            propChangeParams.scope = NativeMethods.DICS_FLAG_GLOBAL;
            propChangeParams.hwProfile = 0;
            success = NativeMethods.SetupDiSetClassInstallParams(deviceInfoSet, ref deviceInfoData, ref propChangeParams, Marshal.SizeOf(propChangeParams));
            if (!success)
            {
                throw new Exception("Error setting class install params, error code = " + Marshal.GetLastWin32Error());
            }
            success = NativeMethods.SetupDiCallClassInstaller(NativeMethods.DIF_PROPERTYCHANGE, deviceInfoSet, ref deviceInfoData);
            // TEST: If previous SetupDiCallClassInstaller fails, just continue
            // otherwise device will likely get permanently disabled.
            

            //System.Threading.Thread.Sleep(50);
            sw.Restart();
            while (sw.ElapsedMilliseconds < 50)
            {
                // Use SpinWait to keep control of current thread. Using Sleep could potentially
                // cause other events to get run out of order
                System.Threading.Thread.SpinWait(100);
            }
            sw.Stop();

            propChangeParams.stateChange = NativeMethods.DICS_ENABLE;
            success = NativeMethods.SetupDiSetClassInstallParams(deviceInfoSet, ref deviceInfoData, ref propChangeParams, Marshal.SizeOf(propChangeParams));
            if (!success)
            {
                throw new Exception("Error setting class install params, error code = " + Marshal.GetLastWin32Error());
            }
            success = NativeMethods.SetupDiCallClassInstaller(NativeMethods.DIF_PROPERTYCHANGE, deviceInfoSet, ref deviceInfoData);
            if (!success)
            {
                throw new Exception("Error enabling device, error code = " + Marshal.GetLastWin32Error());
            }

            //System.Threading.Thread.Sleep(50);
            sw.Restart();
            while (sw.ElapsedMilliseconds < 50)
            {
                // Use SpinWait to keep control of current thread. Using Sleep could potentially
                // cause other events to get run out of order
                System.Threading.Thread.SpinWait(100);
            }
            sw.Stop();

            NativeMethods.SetupDiDestroyDeviceInfoList(deviceInfoSet);
        }
    }
    //SIXAXIS
    public class SixAxisEventArgs : EventArgs
    {
        public readonly SixAxis sixAxis;
        public readonly DateTime timeStamp;
        public SixAxisEventArgs(DateTime utcTimestamp, SixAxis sa)
        {
            sixAxis = sa;
            timeStamp = utcTimestamp;
        }
    }

    public class SixAxis
    {
        public const int ACC_RES_PER_G = 8192;
        private const float F_ACC_RES_PER_G = ACC_RES_PER_G;
        public const int GYRO_RES_IN_DEG_SEC = 16;
        private const float F_GYRO_RES_IN_DEG_SEC = GYRO_RES_IN_DEG_SEC;

        public int gyroYaw, gyroPitch, gyroRoll, accelX, accelY, accelZ;
        public int outputAccelX, outputAccelY, outputAccelZ;
        public double accelXG, accelYG, accelZG;
        public double angVelYaw, angVelPitch, angVelRoll;
        public int gyroYawFull, gyroPitchFull, gyroRollFull;
        public int accelXFull, accelYFull, accelZFull;
        public double elapsed;
        public SixAxis previousAxis = null;

        private double tempDouble = 0d;

        public SixAxis(int X, int Y, int Z,
            int aX, int aY, int aZ,
            double elapsedDelta, SixAxis prevAxis = null)
        {
            populate(X, Y, Z, aX, aY, aZ, elapsedDelta, prevAxis);
        }

        public void copy(SixAxis src)
        {
            gyroYaw = src.gyroYaw;
            gyroPitch = src.gyroPitch;
            gyroRoll = src.gyroRoll;

            gyroYawFull = src.gyroYawFull;
            accelXFull = src.accelXFull; accelYFull = src.accelYFull; accelZFull = src.accelZFull;

            angVelYaw = src.angVelYaw;
            angVelPitch = src.angVelPitch;
            angVelRoll = src.angVelRoll;

            accelXG = src.accelXG;
            accelYG = src.accelYG;
            accelZG = src.accelZG;

            // Put accel ranges between 0 - 128 abs
            accelX = src.accelX;
            accelY = src.accelY;
            accelZ = src.accelZ;
            outputAccelX = accelX;
            outputAccelY = accelY;
            outputAccelZ = accelZ;

            elapsed = src.elapsed;
            previousAxis = src.previousAxis;
        }

        public void populate(int X, int Y, int Z,
            int aX, int aY, int aZ,
            double elapsedDelta, SixAxis prevAxis = null)
        {
            gyroYaw = -X / 256;
            gyroPitch = Y / 256;
            gyroRoll = -Z / 256;

            gyroYawFull = -X; gyroPitchFull = Y; gyroRollFull = -Z;
            accelXFull = -aX; accelYFull = -aY; accelZFull = aZ;

            angVelYaw = gyroYawFull / F_GYRO_RES_IN_DEG_SEC;
            angVelPitch = gyroPitchFull / F_GYRO_RES_IN_DEG_SEC;
            angVelRoll = gyroRollFull / F_GYRO_RES_IN_DEG_SEC;

            accelXG = tempDouble = accelXFull / F_ACC_RES_PER_G;
            accelYG = tempDouble = accelYFull / F_ACC_RES_PER_G;
            accelZG = tempDouble = accelZFull / F_ACC_RES_PER_G;

            // Put accel ranges between 0 - 128 abs
            accelX = -aX / 64;
            accelY = -aY / 64;
            accelZ = aZ / 64;
            outputAccelX = accelX;
            outputAccelY = accelY;
            outputAccelZ = accelZ;

            elapsed = elapsedDelta;
            previousAxis = prevAxis;
        }
    }

    internal class CalibData
    {
        public int bias;
        public int sensNumer;
        public int sensDenom;
        public const int GyroPitchIdx = 0, GyroYawIdx = 1, GyroRollIdx = 2,
        AccelXIdx = 3, AccelYIdx = 4, AccelZIdx = 5;
    }

    public class DS4SixAxis
    {
        public event EventHandler<SixAxisEventArgs> SixAccelMoved = null;
        private SixAxis sPrev = null, now = null;
        private CalibData[] calibrationData = new CalibData[6] { new CalibData(), new CalibData(),
            new CalibData(), new CalibData(), new CalibData(), new CalibData()
        };

        public DS4SixAxis()
        {
            sPrev = new SixAxis(0, 0, 0, 0, 0, 0, 0.0);
            now = new SixAxis(0, 0, 0, 0, 0, 0, 0.0);
        }

        int temInt = 0;
        public void setCalibrationData(ref byte[] calibData, bool fromUSB)
        {
            int pitchPlus, pitchMinus, yawPlus, yawMinus, rollPlus, rollMinus,
                accelXPlus, accelXMinus, accelYPlus, accelYMinus, accelZPlus, accelZMinus,
                gyroSpeedPlus, gyroSpeedMinus;

            calibrationData[0].bias = (short)((ushort)(calibData[2] << 8) | calibData[1]);
            calibrationData[1].bias = (short)((ushort)(calibData[4] << 8) | calibData[3]);
            calibrationData[2].bias = (short)((ushort)(calibData[6] << 8) | calibData[5]);

            if (!fromUSB)
            {
                pitchPlus = temInt = (short)((ushort)(calibData[8] << 8) | calibData[7]);
                yawPlus = temInt = (short)((ushort)(calibData[10] << 8) | calibData[9]);
                rollPlus = temInt = (short)((ushort)(calibData[12] << 8) | calibData[11]);
                pitchMinus = temInt = (short)((ushort)(calibData[14] << 8) | calibData[13]);
                yawMinus = temInt = (short)((ushort)(calibData[16] << 8) | calibData[15]);
                rollMinus = temInt = (short)((ushort)(calibData[18] << 8) | calibData[17]);
            }
            else
            {
                pitchPlus = temInt = (short)((ushort)(calibData[8] << 8) | calibData[7]);
                pitchMinus = temInt = (short)((ushort)(calibData[10] << 8) | calibData[9]);
                yawPlus = temInt = (short)((ushort)(calibData[12] << 8) | calibData[11]);
                yawMinus = temInt = (short)((ushort)(calibData[14] << 8) | calibData[13]);
                rollPlus = temInt = (short)((ushort)(calibData[16] << 8) | calibData[15]);
                rollMinus = temInt = (short)((ushort)(calibData[18] << 8) | calibData[17]);
            }

            gyroSpeedPlus = temInt = (short)((ushort)(calibData[20] << 8) | calibData[19]);
            gyroSpeedMinus = temInt = (short)((ushort)(calibData[22] << 8) | calibData[21]);
            accelXPlus = temInt = (short)((ushort)(calibData[24] << 8) | calibData[23]);
            accelXMinus = temInt = (short)((ushort)(calibData[26] << 8) | calibData[25]);

            accelYPlus = temInt = (short)((ushort)(calibData[28] << 8) | calibData[27]);
            accelYMinus = temInt = (short)((ushort)(calibData[30] << 8) | calibData[29]);

            accelZPlus = temInt = (short)((ushort)(calibData[32] << 8) | calibData[31]);
            accelZMinus = temInt = (short)((ushort)(calibData[34] << 8) | calibData[33]);

            int gyroSpeed2x = temInt = (gyroSpeedPlus + gyroSpeedMinus);
            calibrationData[0].sensNumer = gyroSpeed2x * SixAxis.GYRO_RES_IN_DEG_SEC;
            calibrationData[0].sensDenom = pitchPlus - pitchMinus;

            calibrationData[1].sensNumer = gyroSpeed2x * SixAxis.GYRO_RES_IN_DEG_SEC;
            calibrationData[1].sensDenom = yawPlus - yawMinus;

            calibrationData[2].sensNumer = gyroSpeed2x * SixAxis.GYRO_RES_IN_DEG_SEC;
            calibrationData[2].sensDenom = rollPlus - rollMinus;

            int accelRange = temInt = accelXPlus - accelXMinus;
            calibrationData[3].bias = accelXPlus - accelRange / 2;
            calibrationData[3].sensNumer = 2 * SixAxis.ACC_RES_PER_G;
            calibrationData[3].sensDenom = accelRange;

            accelRange = temInt = accelYPlus - accelYMinus;
            calibrationData[4].bias = accelYPlus - accelRange / 2;
            calibrationData[4].sensNumer = 2 * SixAxis.ACC_RES_PER_G;
            calibrationData[4].sensDenom = accelRange;

            accelRange = temInt = accelZPlus - accelZMinus;
            calibrationData[5].bias = accelZPlus - accelRange / 2;
            calibrationData[5].sensNumer = 2 * SixAxis.ACC_RES_PER_G;
            calibrationData[5].sensDenom = accelRange;
        }

        private void applyCalibs(ref int yaw, ref int pitch, ref int roll,
            ref int accelX, ref int accelY, ref int accelZ)
        {
            CalibData current = calibrationData[0];
            temInt = pitch - current.bias;
            pitch = temInt = (int)(temInt * (current.sensNumer / (float)current.sensDenom));

            current = calibrationData[1];
            temInt = yaw - current.bias;
            yaw = temInt = (int)(temInt * (current.sensNumer / (float)current.sensDenom));

            current = calibrationData[2];
            temInt = roll - current.bias;
            roll = temInt = (int)(temInt * (current.sensNumer / (float)current.sensDenom));

            current = calibrationData[3];
            temInt = accelX - current.bias;
            accelX = temInt = (int)(temInt * (current.sensNumer / (float)current.sensDenom));

            current = calibrationData[4];
            temInt = accelY - current.bias;
            accelY = temInt = (int)(temInt * (current.sensNumer / (float)current.sensDenom));

            current = calibrationData[5];
            temInt = accelZ - current.bias;
            accelZ = temInt = (int)(temInt * (current.sensNumer / (float)current.sensDenom));
        }

        public void handleSixaxis(byte[] gyro, byte[] accel, DS4State state,
            double elapsedDelta)
        {
            int currentYaw = (short)((ushort)(gyro[3] << 8) | gyro[2]);
            int currentPitch = (short)((ushort)(gyro[1] << 8) | gyro[0]);
            int currentRoll = (short)((ushort)(gyro[5] << 8) | gyro[4]);
            int AccelX = (short)((ushort)(accel[1] << 8) | accel[0]);
            int AccelY = (short)((ushort)(accel[3] << 8) | accel[2]);
            int AccelZ = (short)((ushort)(accel[5] << 8) | accel[4]);

            applyCalibs(ref currentYaw, ref currentPitch, ref currentRoll, ref AccelX, ref AccelY, ref AccelZ);

            SixAxisEventArgs args = null;
            if (AccelX != 0 || AccelY != 0 || AccelZ != 0)
            {
                if (SixAccelMoved != null)
                {
                    sPrev.copy(now);
                    now.populate(currentYaw, currentPitch, currentRoll,
                        AccelX, AccelY, AccelZ, elapsedDelta, sPrev);

                    args = new SixAxisEventArgs(state.ReportTimeStamp, now);
                    state.Motion = now;
                    SixAccelMoved(this, args);
                }
            }
        }
    }
    //Crc32
    public sealed class Crc32Algorithm : HashAlgorithm
    {
        public const uint DefaultPolynomial = 0xedb88320u;
        public const uint DefaultSeed = 0xffffffffu;

        private static readonly uint[] defaultTable =
        {
            0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA,
            0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
            0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
            0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
            0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE,
            0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
            0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC,
            0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
            0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
            0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
            0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940,
            0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
            0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116,
            0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
            0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
            0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
            0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A,
            0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
            0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818,
            0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
            0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
            0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
            0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C,
            0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
            0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2,
            0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
            0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
            0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
            0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086,
            0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
            0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4,
            0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
            0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
            0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
            0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8,
            0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
            0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE,
            0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
            0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
            0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
            0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252,
            0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
            0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60,
            0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
            0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
            0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
            0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04,
            0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
            0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A,
            0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
            0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
            0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
            0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E,
            0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
            0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C,
            0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
            0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
            0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
            0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0,
            0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
            0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6,
            0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
            0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
            0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
        };

        private static uint[] testLook = new uint[16 * 256];
        private static bool secondTablePop = false;

        private readonly uint _seed;
        private readonly uint[] _table;
        private uint _hash;

        public Crc32Algorithm()
            : this(DefaultPolynomial, DefaultSeed)
        {
        }

        public Crc32Algorithm(uint polynomial, uint seed)
        {
            _table = InitializeTable(polynomial);
            _seed = _hash = seed;
        }

        public override void Initialize()
        {
            _hash = _seed;
        }

        protected override void HashCore(byte[] buffer, int start, int length)
        {
            _hash = CalculateHash(_table, _hash, buffer, start, length);
        }

        protected override byte[] HashFinal()
        {
            var hashBuffer = UintToBigEndianBytes(~_hash);
            HashValue = hashBuffer;
            return hashBuffer;
        }

        public override int HashSize { get { return 32; } }

        public static uint Compute(byte[] buffer)
        {
            return ~CalculateHash(defaultTable, DefaultSeed, buffer, 0, buffer.Length);
        }

        public static uint Compute(uint seed, byte[] buffer)
        {
            return ~CalculateHash(defaultTable, seed, buffer, 0, buffer.Length);
        }

        public static uint Compute(uint polynomial, uint seed, byte[] buffer)
        {
            return ~CalculateHash(InitializeTable(polynomial), seed, buffer, 0, buffer.Length);
        }

        public static uint[] InitializeTable(uint polynomial)
        {
            if (polynomial == DefaultPolynomial)
            {
                if (!secondTablePop)
                {
                    for (int i = 0; i <= 0xFF; i++)
                    {
                        testLook[0 + i] = defaultTable[i];
                        testLook[256 + i] = (defaultTable[i] >> 8) ^ defaultTable[defaultTable[i] & 0xFF];
                        testLook[512 + i] = (testLook[256 + i] >> 8) ^ defaultTable[testLook[256 + i] & 0xFF];
                        testLook[768 + i] = (testLook[512 + i] >> 8) ^ defaultTable[testLook[512 + i] & 0xFF];

                        testLook[1024 + i] = (testLook[768 + i] >> 8) ^ defaultTable[testLook[768 + i] & 0xFF];
                        testLook[1280 + i] = (testLook[1024 + i] >> 8) ^ defaultTable[testLook[1024 + i] & 0xFF];
                        testLook[1536 + i] = (testLook[1280 + i] >> 8) ^ defaultTable[testLook[1280 + i] & 0xFF];
                        testLook[1792 + i] = (testLook[1536 + i] >> 8) ^ defaultTable[testLook[1536 + i] & 0xFF];

                        testLook[2048 + i] = (testLook[1792 + i] >> 8) ^ defaultTable[testLook[1792 + i] & 0xFF];
                        testLook[2304 + i] = (testLook[2048 + i] >> 8) ^ defaultTable[testLook[2048 + i] & 0xFF];
                        testLook[2560 + i] = (testLook[2304 + i] >> 8) ^ defaultTable[testLook[2304 + i] & 0xFF];
                        testLook[2816 + i] = (testLook[2560 + i] >> 8) ^ defaultTable[testLook[2560 + i] & 0xFF];

                        testLook[3072 + i] = (testLook[2816 + i] >> 8) ^ defaultTable[testLook[2816 + i] & 0xFF];
                        testLook[3328 + i] = (testLook[3072 + i] >> 8) ^ defaultTable[testLook[3072 + i] & 0xFF];
                        testLook[3584 + i] = (testLook[3328 + i] >> 8) ^ defaultTable[testLook[3328 + i] & 0xFF];
                        testLook[3840 + i] = (testLook[3584 + i] >> 8) ^ defaultTable[testLook[3584 + i] & 0xFF];
                    }

                    secondTablePop = true;
                }

                return defaultTable;
            }


            var createTable = new uint[256];
            for (uint i = 0; i < 256; i++)
            {
                var entry = i;
                for (var j = 0; j < 8; j++)
                    entry = ((entry & 1) == 1) ? (entry >> 1) ^ polynomial : (entry >> 1);
                createTable[i] = entry;
            }

            return createTable;
        }

        private static uint CalculateHash(uint[] table, uint seed, byte[] buffer, int start, int size)
        {
            var crc = seed;
            for (var i = start; i < size - start; i++)
                crc = (crc >> 8) ^ table[buffer[i] ^ crc & 0xff];
            return crc;
        }

        public static unsafe uint CalculateBasicHash(ref uint seed, ref byte[] buffer, int offset, int size)
        {
            uint crc = seed;
            int i = offset;

            fixed (byte* byteP = buffer)
            fixed (uint* byteT = testLook)
            {
                while (size >= 16)
                {
                    uint one = (byteP[i++] |
                                (uint)(byteP[i++] << 8) |
                                (uint)(byteP[i++] << 16) |
                                (uint)(byteP[i++] << 24)) ^ crc;
                    uint two = byteP[i++] |
                                (uint)(byteP[i++] << 8) |
                                (uint)(byteP[i++] << 16) |
                                (uint)(byteP[i++] << 24);
                    uint three = (byteP[i++] |
                                (uint)(byteP[i++] << 8) |
                                (uint)(byteP[i++] << 16) |
                                (uint)(byteP[i++] << 24));
                    uint four = byteP[i++] |
                                (uint)(byteP[i++] << 8) |
                                (uint)(byteP[i++] << 16) |
                                (uint)(byteP[i++] << 24);

                    crc = byteT[3840 + (one & 0xFF)] ^
                        byteT[3584 + ((one >> 8) & 0xFF)] ^
                        byteT[3328 + ((one >> 16) & 0xFF)] ^
                        byteT[3072 + ((one >> 24) & 0xFF)] ^
                        byteT[2816 + (two & 0xFF)] ^
                        byteT[2560 + ((two >> 8) & 0xFF)] ^
                        byteT[2304 + ((two >> 16) & 0xFF)] ^
                        byteT[2048 + ((two >> 24) & 0xFF)] ^
                        byteT[1792 + (three & 0xFF)] ^
                        byteT[1536 + ((three >> 8) & 0xFF)] ^
                        byteT[1280 + ((three >> 16) & 0xFF)] ^
                        byteT[1024 + ((three >> 24) & 0xFF)] ^
                        byteT[768 + (four & 0xFF)] ^
                        byteT[512 + ((four >> 8) & 0xFF)] ^
                        byteT[256 + ((four >> 16) & 0xFF)] ^
                        byteT[(four >> 24) & 0xFF];

                    size -= 16;
                }

                while (size >= 8)
                {
                    uint one8 = (byteP[i++] |
                                (uint)(byteP[i++] << 8) |
                                (uint)(byteP[i++] << 16) |
                                (uint)(byteP[i++] << 24)) ^ crc;
                    uint two8 = byteP[i++] |
                                (uint)(byteP[i++] << 8) |
                                (uint)(byteP[i++] << 16) |
                                (uint)(byteP[i++] << 24);
                    crc = byteT[1792 + (one8 & 0xFF)] ^
                        byteT[1536 + ((one8 >> 8) & 0xFF)] ^
                        byteT[1280 + ((one8 >> 16) & 0xFF)] ^
                        byteT[1024 + (one8 >> 24)] ^
                        byteT[768 + (two8 & 0xFF)] ^
                        byteT[512 + ((two8 >> 8) & 0xFF)] ^
                        byteT[256 + ((two8 >> 16) & 0xFF)] ^
                        byteT[two8 >> 24];

                    size -= 8;
                }

                while (--size >= 0)
                {
                    crc = (crc >> 8) ^ byteT[(crc & 0xFF) ^ byteP[i++]];// i++;
                }
            }

            return crc;
        }

        public static unsafe uint CalculateFasterBTHash(ref uint seed, ref byte[] buffer, ref int start, ref int size)
        {

            uint crc = seed;
            int i = start;
            int bufsize = size;
            //while (bufsize >= 16)
            fixed (byte* byteP = buffer)
            fixed (uint* byteT = testLook)
            {
                for (int j = 0; j < 4; j++)
                {
                    uint one = (byteP[i++] |
                                (uint)(byteP[i++] << 8) |
                                (uint)(byteP[i++] << 16) |
                                (uint)(byteP[i++] << 24)) ^ crc;
                    uint two = byteP[i++] |
                                (uint)(byteP[i++] << 8) |
                                (uint)(byteP[i++] << 16) |
                                (uint)(byteP[i++] << 24);
                    uint three = (byteP[i++] |
                                (uint)(byteP[i++] << 8) |
                                (uint)(byteP[i++] << 16) |
                                (uint)(byteP[i++] << 24));
                    uint four = byteP[i++] |
                                (uint)(byteP[i++] << 8) |
                                (uint)(byteP[i++] << 16) |
                                (uint)(byteP[i++] << 24);

                    crc = byteT[3840 + (one & 0xFF)] ^
                        byteT[3584 + ((one >> 8) & 0xFF)] ^
                        byteT[3328 + ((one >> 16) & 0xFF)] ^
                        byteT[3072 + ((one >> 24) & 0xFF)] ^
                        byteT[2816 + (two & 0xFF)] ^
                        byteT[2560 + ((two >> 8) & 0xFF)] ^
                        byteT[2304 + ((two >> 16) & 0xFF)] ^
                        byteT[2048 + ((two >> 24) & 0xFF)] ^
                        byteT[1792 + (three & 0xFF)] ^
                        byteT[1536 + ((three >> 8) & 0xFF)] ^
                        byteT[1280 + ((three >> 16) & 0xFF)] ^
                        byteT[1024 + ((three >> 24) & 0xFF)] ^
                        byteT[768 + (four & 0xFF)] ^
                        byteT[512 + ((four >> 8) & 0xFF)] ^
                        byteT[256 + ((four >> 16) & 0xFF)] ^
                        byteT[(four >> 24) & 0xFF];

                    bufsize -= 16;
                }

                //while (bufsize >= 8)
                //if (bufsize >= 8)

                uint one8 = (byteP[i++] |
                            (uint)(byteP[i++] << 8) |
                            (uint)(byteP[i++] << 16) |
                            (uint)(byteP[i++] << 24)) ^ crc;
                uint two8 = byteP[i++] |
                            (uint)(byteP[i++] << 8) |
                            (uint)(byteP[i++] << 16) |
                            (uint)(byteP[i++] << 24);
                crc = byteT[1792 + (one8 & 0xFF)] ^
                    byteT[1536 + ((one8 >> 8) & 0xFF)] ^
                    byteT[1280 + ((one8 >> 16) & 0xFF)] ^
                    byteT[1024 + (one8 >> 24)] ^
                    byteT[768 + (two8 & 0xFF)] ^
                    byteT[512 + ((two8 >> 8) & 0xFF)] ^
                    byteT[256 + ((two8 >> 16) & 0xFF)] ^
                    byteT[two8 >> 24];

                bufsize -= 8;


                //while (--bufsize >= 0)
                //{
                crc = (crc >> 8) ^ byteT[(crc & 0xFF) ^ byteP[i++]];// i++;
                crc = (crc >> 8) ^ byteT[(crc & 0xFF) ^ byteP[i++]];// i++;
                                                                    //}
            }

            return crc;
        }

        private static byte[] UintToBigEndianBytes(uint uint32)
        {
            var result = BitConverter.GetBytes(uint32);
            if (BitConverter.IsLittleEndian)
                Array.Reverse(result);
            return result;
        }
    }
    //HIDDEVICE
    public class HidDevice : IDisposable
    {
        public enum ReadStatus
        {
            Success = 0,
            WaitTimedOut = 1,
            WaitFail = 2,
            NoDataRead = 3,
            ReadError = 4,
            NotConnected = 5
        }

        private readonly string _description;
        private readonly string _devicePath;
        private readonly HidDeviceAttributes _deviceAttributes;

        private readonly HidDeviceCapabilities _deviceCapabilities;
        private bool _monitorDeviceEvents;
        private string serial = null;
        internal HidDevice(string devicePath, string description = null)
        {
            _devicePath = devicePath;
            _description = description;

            try
            {
                var hidHandle = OpenHandle(_devicePath, false);

                _deviceAttributes = GetDeviceAttributes(hidHandle);
                _deviceCapabilities = GetDeviceCapabilities(hidHandle);

                hidHandle.Close();
            }
            catch (Exception exception)
            {
                Debug.WriteLine(exception.Message);
                throw new Exception(string.Format("Error querying HID device '{0}'.", devicePath), exception);
            }
        }

        public SafeFileHandle safeReadHandle { get; private set; }
        public FileStream fileStream { get; private set; }
        public bool IsOpen { get; private set; }
        public bool IsExclusive { get; private set; }
        public bool IsConnected { get { return HidDevices.IsConnected(_devicePath); } }
        public string Description { get { return _description; } }
        public HidDeviceCapabilities Capabilities { get { return _deviceCapabilities; } }
        public HidDeviceAttributes Attributes { get { return _deviceAttributes; } }
        public string DevicePath { get { return _devicePath; } }

        public override string ToString()
        {
            return string.Format("VendorID={0}, ProductID={1}, Version={2}, DevicePath={3}",
                                _deviceAttributes.VendorHexId,
                                _deviceAttributes.ProductHexId,
                                _deviceAttributes.Version,
                                _devicePath);
        }

        public void OpenDevice(bool isExclusive)
        {
            if (IsOpen) return;
            try
            {
                if (safeReadHandle == null || safeReadHandle.IsInvalid)
                    safeReadHandle = OpenHandle(_devicePath, isExclusive);
            }
            catch (Exception exception)
            {
                IsOpen = false;
                throw new Exception("Error opening HID device.", exception);
            }

            IsOpen = !safeReadHandle.IsInvalid;
            IsExclusive = isExclusive;
        }

        public void OpenFileStream(int reportSize)
        {
            if (fileStream == null && !safeReadHandle.IsInvalid)
            {
                fileStream = new FileStream(safeReadHandle, FileAccess.ReadWrite, reportSize, true);
            }
        }

        public bool IsFileStreamOpen()
        {
            bool result = false;
            if (fileStream != null)
            {
                result = !fileStream.SafeFileHandle.IsInvalid && !fileStream.SafeFileHandle.IsClosed;
            }

            return result;
        }

        public void CloseDevice()
        {
            if (!IsOpen) return;
            closeFileStreamIO();

            IsOpen = false;
        }

        public void Dispose()
        {
            CancelIO();
            CloseDevice();
        }

        public void CancelIO()
        {
            if (IsOpen)
                NativeMethods.CancelIoEx(safeReadHandle.DangerousGetHandle(), IntPtr.Zero);
        }

        public bool ReadInputReport(byte[] data)
        {
            if (safeReadHandle == null)
                safeReadHandle = OpenHandle(_devicePath, true);
            return NativeMethods.HidD_GetInputReport(safeReadHandle, data, data.Length);
        }

        public bool WriteFeatureReport(byte[] data)
        {
            bool result = false;
            if (IsOpen && safeReadHandle != null)
            {
                result = NativeMethods.HidD_SetFeature(safeReadHandle, data, data.Length);
            }

            return result;
        }


        private static HidDeviceAttributes GetDeviceAttributes(SafeFileHandle hidHandle)
        {
            var deviceAttributes = default(NativeMethods.HIDD_ATTRIBUTES);
            deviceAttributes.Size = Marshal.SizeOf(deviceAttributes);
            NativeMethods.HidD_GetAttributes(hidHandle.DangerousGetHandle(), ref deviceAttributes);
            return new HidDeviceAttributes(deviceAttributes);
        }

        private static HidDeviceCapabilities GetDeviceCapabilities(SafeFileHandle hidHandle)
        {
            var capabilities = default(NativeMethods.HIDP_CAPS);
            var preparsedDataPointer = default(IntPtr);

            if (NativeMethods.HidD_GetPreparsedData(hidHandle.DangerousGetHandle(), ref preparsedDataPointer))
            {
                NativeMethods.HidP_GetCaps(preparsedDataPointer, ref capabilities);
                NativeMethods.HidD_FreePreparsedData(preparsedDataPointer);
            }

            return new HidDeviceCapabilities(capabilities);
        }

        private void closeFileStreamIO()
        {
            if (fileStream != null)
            {
                try
                {
                    fileStream.Close();
                }
                catch (IOException) { }
                catch (OperationCanceledException) { }
            }

            fileStream = null;
            Debug.WriteLine("Close fs");
            if (safeReadHandle != null && !safeReadHandle.IsInvalid)
            {
                try
                {
                    if (!safeReadHandle.IsClosed)
                    {
                        safeReadHandle.Close();
                        Debug.WriteLine("Close sh");
                    }
                }
                catch (IOException) { }
            }

            safeReadHandle = null;
        }

        public void flush_Queue()
        {
            if (safeReadHandle != null)
            {
                NativeMethods.HidD_FlushQueue(safeReadHandle);
            }
        }

        private ReadStatus ReadWithFileStreamTask(byte[] inputBuffer)
        {
            try
            {
                if (fileStream.Read(inputBuffer, 0, inputBuffer.Length) > 0)
                {
                    return ReadStatus.Success;
                }
                else
                {
                    return ReadStatus.NoDataRead;
                }
            }
            catch (Exception)
            {
                return ReadStatus.ReadError;
            }
        }

        public ReadStatus ReadFile(byte[] inputBuffer)
        {
            if (safeReadHandle == null)
                safeReadHandle = OpenHandle(_devicePath, true);
            try
            {
                uint bytesRead;
                if (NativeMethods.ReadFile(safeReadHandle.DangerousGetHandle(), inputBuffer, (uint)inputBuffer.Length, out bytesRead, IntPtr.Zero))
                {
                    return ReadStatus.Success;
                }
                else
                {
                    return ReadStatus.NoDataRead;
                }
            }
            catch (Exception)
            {
                return ReadStatus.ReadError;
            }
        }

        public ReadStatus ReadWithFileStream(byte[] inputBuffer)
        {
            try
            {
                if (fileStream.Read(inputBuffer, 0, inputBuffer.Length) > 0)
                {
                    return ReadStatus.Success;
                }
                else
                {
                    return ReadStatus.NoDataRead;
                }
            }
            catch (Exception)
            {
                return ReadStatus.ReadError;
            }
        }

        public ReadStatus ReadWithFileStream(byte[] inputBuffer, int timeout)
        {
            try
            {
                if (safeReadHandle == null)
                    safeReadHandle = OpenHandle(_devicePath, true);
                if (fileStream == null && !safeReadHandle.IsInvalid)
                    fileStream = new FileStream(safeReadHandle, FileAccess.ReadWrite, inputBuffer.Length, true);

                if (!safeReadHandle.IsInvalid && fileStream.CanRead)
                {
                    Task<ReadStatus> readFileTask = new Task<ReadStatus>(() => ReadWithFileStreamTask(inputBuffer));
                    readFileTask.Start();
                    bool success = readFileTask.Wait(timeout);
                    if (success)
                    {
                        if (readFileTask.Result == ReadStatus.Success)
                        {
                            return ReadStatus.Success;
                        }
                        else if (readFileTask.Result == ReadStatus.ReadError)
                        {
                            return ReadStatus.ReadError;
                        }
                        else if (readFileTask.Result == ReadStatus.NoDataRead)
                        {
                            return ReadStatus.NoDataRead;
                        }
                    }
                    else
                        return ReadStatus.WaitTimedOut;
                }

            }
            catch (Exception e)
            {
                if (e is AggregateException)
                {
                    Debug.WriteLine(e.Message);
                    return ReadStatus.WaitFail;
                }
                else
                {
                    return ReadStatus.ReadError;
                }
            }

            return ReadStatus.ReadError;
        }

        public ReadStatus ReadAsyncWithFileStream(byte[] inputBuffer, int timeout)
        {
            try
            {
                if (safeReadHandle == null)
                    safeReadHandle = OpenHandle(_devicePath, true);
                if (fileStream == null && !safeReadHandle.IsInvalid)
                    fileStream = new FileStream(safeReadHandle, FileAccess.ReadWrite, inputBuffer.Length, true);

                if (!safeReadHandle.IsInvalid && fileStream.CanRead)
                {
                    Task<int> readTask = fileStream.ReadAsync(inputBuffer, 0, inputBuffer.Length);
                    bool success = readTask.Wait(timeout);
                    if (success)
                    {
                        if (readTask.Result > 0)
                        {
                            return ReadStatus.Success;
                        }
                        else
                        {
                            return ReadStatus.NoDataRead;
                        }
                    }
                    else
                    {
                        return ReadStatus.WaitTimedOut;
                    }
                }

            }
            catch (Exception e)
            {
                if (e is AggregateException)
                {
                    Debug.WriteLine(e.Message);
                    return ReadStatus.WaitFail;
                }
                else
                {
                    return ReadStatus.ReadError;
                }
            }

            return ReadStatus.ReadError;
        }

        public bool WriteOutputReportViaControl(byte[] outputBuffer)
        {
            if (safeReadHandle == null)
            {
                safeReadHandle = OpenHandle(_devicePath, true);
            }

            if (NativeMethods.HidD_SetOutputReport(safeReadHandle, outputBuffer, outputBuffer.Length))
                return true;
            else
                return false;
        }

        private bool WriteOutputReportViaInterruptTask(byte[] outputBuffer)
        {
            try
            {
                fileStream.Write(outputBuffer, 0, outputBuffer.Length);
                return true;
            }
            catch (Exception e)
            {
                Debug.WriteLine(e.Message);
                return false;
            }
        }

        public bool WriteOutputReportViaInterrupt(byte[] outputBuffer, int timeout)
        {
            try
            {
                if (safeReadHandle == null)
                {
                    safeReadHandle = OpenHandle(_devicePath, true);
                }
                if (fileStream == null && !safeReadHandle.IsInvalid)
                {
                    fileStream = new FileStream(safeReadHandle, FileAccess.ReadWrite, outputBuffer.Length, true);
                }
                if (fileStream != null && fileStream.CanWrite && !safeReadHandle.IsInvalid)
                {
                    fileStream.Write(outputBuffer, 0, outputBuffer.Length);
                    return true;
                }
                else
                {
                    return false;
                }
            }
            catch (Exception)
            {
                return false;
            }

        }

        public bool WriteAsyncOutputReportViaInterrupt(byte[] outputBuffer)
        {
            try
            {
                if (safeReadHandle == null)
                {
                    safeReadHandle = OpenHandle(_devicePath, true);
                }
                if (fileStream == null && !safeReadHandle.IsInvalid)
                {
                    fileStream = new FileStream(safeReadHandle, FileAccess.ReadWrite, outputBuffer.Length, true);
                }
                if (fileStream != null && fileStream.CanWrite && !safeReadHandle.IsInvalid)
                {
                    Task writeTask = fileStream.WriteAsync(outputBuffer, 0, outputBuffer.Length);
                    //fileStream.Write(outputBuffer, 0, outputBuffer.Length);
                    return true;
                }
                else
                {
                    return false;
                }
            }
            catch (Exception)
            {
                return false;
            }

        }

        private SafeFileHandle OpenHandle(String devicePathName, Boolean isExclusive)
        {
            SafeFileHandle hidHandle;

            if (isExclusive)
            {
                hidHandle = NativeMethods.CreateFile(devicePathName, NativeMethods.GENERIC_READ | NativeMethods.GENERIC_WRITE, 0, IntPtr.Zero, NativeMethods.OpenExisting, 0x20000000 | 0x80000000 | NativeMethods.FILE_FLAG_OVERLAPPED, 0);
            }
            else
            {
                hidHandle = NativeMethods.CreateFile(devicePathName, NativeMethods.GENERIC_READ | NativeMethods.GENERIC_WRITE, NativeMethods.FILE_SHARE_READ | NativeMethods.FILE_SHARE_WRITE, IntPtr.Zero, NativeMethods.OpenExisting, 0x20000000 | 0x80000000 | NativeMethods.FILE_FLAG_OVERLAPPED, 0);
            }

            return hidHandle;
        }

        public bool readFeatureData(byte[] inputBuffer)
        {
            return NativeMethods.HidD_GetFeature(safeReadHandle.DangerousGetHandle(), inputBuffer, inputBuffer.Length);
        }

        public void resetSerial()
        {
            serial = null;
        }

        public string readSerial()
        {
            if (serial != null)
                return serial;

            if (Capabilities.InputReportByteLength == 64)
            {
                byte[] buffer = new byte[16];
                buffer[0] = 18;
                readFeatureData(buffer);
                serial = String.Format("{0:X02}:{1:X02}:{2:X02}:{3:X02}:{4:X02}:{5:X02}", buffer[6], buffer[5], buffer[4], buffer[3], buffer[2], buffer[1]);
                return serial;
            }
            else
            {
                byte[] buffer = new byte[126];
#if WIN64
                ulong bufferLen = 126;
#else
                uint bufferLen = 126;
#endif
                NativeMethods.HidD_GetSerialNumberString(safeReadHandle.DangerousGetHandle(), buffer, bufferLen);
                string MACAddr = System.Text.Encoding.Unicode.GetString(buffer).Replace("\0", string.Empty).ToUpper();
                MACAddr = $"{MACAddr[0]}{MACAddr[1]}:{MACAddr[2]}{MACAddr[3]}:{MACAddr[4]}{MACAddr[5]}:{MACAddr[6]}{MACAddr[7]}:{MACAddr[8]}{MACAddr[9]}:{MACAddr[10]}{MACAddr[11]}";
                serial = MACAddr;
                return serial;
            }
        }
    }
    public class HidDeviceAttributes
        {
            internal HidDeviceAttributes(NativeMethods.HIDD_ATTRIBUTES attributes)
            {
                VendorId = attributes.VendorID;
                ProductId = attributes.ProductID;
                Version = attributes.VersionNumber;

                VendorHexId = "0x" + attributes.VendorID.ToString("X4");
                ProductHexId = "0x" + attributes.ProductID.ToString("X4");
            }

            public int VendorId { get; private set; }
            public int ProductId { get; private set; }
            public int Version { get; private set; }
            public string VendorHexId { get; set; }
            public string ProductHexId { get; set; }
        }
    public class HidDeviceCapabilities
    {
        internal HidDeviceCapabilities(NativeMethods.HIDP_CAPS capabilities)
        {
            Usage = capabilities.Usage;
            UsagePage = capabilities.UsagePage;
            InputReportByteLength = capabilities.InputReportByteLength;
            OutputReportByteLength = capabilities.OutputReportByteLength;
            FeatureReportByteLength = capabilities.FeatureReportByteLength;
            Reserved = capabilities.Reserved;
            NumberLinkCollectionNodes = capabilities.NumberLinkCollectionNodes;
            NumberInputButtonCaps = capabilities.NumberInputButtonCaps;
            NumberInputValueCaps = capabilities.NumberInputValueCaps;
            NumberInputDataIndices = capabilities.NumberInputDataIndices;
            NumberOutputButtonCaps = capabilities.NumberOutputButtonCaps;
            NumberOutputValueCaps = capabilities.NumberOutputValueCaps;
            NumberOutputDataIndices = capabilities.NumberOutputDataIndices;
            NumberFeatureButtonCaps = capabilities.NumberFeatureButtonCaps;
            NumberFeatureValueCaps = capabilities.NumberFeatureValueCaps;
            NumberFeatureDataIndices = capabilities.NumberFeatureDataIndices;

        }

        public short Usage { get; private set; }
        public short UsagePage { get; private set; }
        public short InputReportByteLength { get; private set; }
        public short OutputReportByteLength { get; private set; }
        public short FeatureReportByteLength { get; private set; }
        public short[] Reserved { get; private set; }
        public short NumberLinkCollectionNodes { get; private set; }
        public short NumberInputButtonCaps { get; private set; }
        public short NumberInputValueCaps { get; private set; }
        public short NumberInputDataIndices { get; private set; }
        public short NumberOutputButtonCaps { get; private set; }
        public short NumberOutputValueCaps { get; private set; }
        public short NumberOutputDataIndices { get; private set; }
        public short NumberFeatureButtonCaps { get; private set; }
        public short NumberFeatureValueCaps { get; private set; }
        public short NumberFeatureDataIndices { get; private set; }
    }
    public class HidDevices
    {
        private static Guid _hidClassGuid = Guid.Empty;

        public static bool IsConnected(string devicePath)
        {
            return EnumerateDevices().Any(x => x.Path == devicePath);
        }

        public static HidDevice GetDevice(string devicePath)
        {
            return Enumerate(devicePath).FirstOrDefault();
        }

        public static IEnumerable<HidDevice> Enumerate()
        {
            return EnumerateDevices().Select(x => new HidDevice(x.Path, x.Description));
        }

        public static IEnumerable<HidDevice> Enumerate(string devicePath)
        {
            return EnumerateDevices().Where(x => x.Path == devicePath).Select(x => new HidDevice(x.Path, x.Description));
        }

        public static IEnumerable<HidDevice> Enumerate(int vendorId, params int[] productIds)
        {
            return EnumerateDevices().Select(x => new HidDevice(x.Path, x.Description)).Where(x => x.Attributes.VendorId == vendorId &&
                                                                                  productIds.Contains(x.Attributes.ProductId));
        }

        public static IEnumerable<HidDevice> Enumerate(int[] vendorIds, params int[] productIds)
        {
            return EnumerateDevices().Select(x => new HidDevice(x.Path, x.Description)).Where(x => vendorIds.Contains(x.Attributes.VendorId) &&
                                                                                  productIds.Contains(x.Attributes.ProductId));
        }

        public static IEnumerable<HidDevice> EnumerateDS4(VidPidInfo[] devInfo)
        {
            List<HidDevice> foundDevs = new List<HidDevice>();
            int devInfoLen = devInfo.Length;
            IEnumerable<DeviceInfo> temp = EnumerateDevices();
            for (int i = 0, len = temp.Count(); i < len; i++)
            {
                DeviceInfo x = temp.ElementAt(i);
                HidDevice tempDev = new HidDevice(x.Path, x.Description);
                bool found = false;
                for (int j = 0; !found && j < devInfoLen; j++)
                {
                    VidPidInfo tempInfo = devInfo[j];
                    if (tempDev.Attributes.VendorId == tempInfo.vid &&
                        tempDev.Attributes.ProductId == tempInfo.pid)
                    {
                        found = true;
                        foundDevs.Add(tempDev);
                    }
                }
            }

            return foundDevs;
        }

        public static IEnumerable<HidDevice> Enumerate(int vendorId)
        {
            return EnumerateDevices().Select(x => new HidDevice(x.Path, x.Description)).Where(x => x.Attributes.VendorId == vendorId);
        }

        private class DeviceInfo { public string Path { get; set; } public string Description { get; set; } }

        private static IEnumerable<DeviceInfo> EnumerateDevices()
        {
            var devices = new List<DeviceInfo>();
            var hidClass = HidClassGuid;
            var deviceInfoSet = NativeMethods.SetupDiGetClassDevs(ref hidClass, null, 0, NativeMethods.DIGCF_PRESENT | NativeMethods.DIGCF_DEVICEINTERFACE);

            if (deviceInfoSet.ToInt64() != NativeMethods.INVALID_HANDLE_VALUE)
            {
                var deviceInfoData = CreateDeviceInfoData();
                var deviceIndex = 0;

                while (NativeMethods.SetupDiEnumDeviceInfo(deviceInfoSet, deviceIndex, ref deviceInfoData))
                {
                    deviceIndex += 1;

                    var deviceInterfaceData = new NativeMethods.SP_DEVICE_INTERFACE_DATA();
                    deviceInterfaceData.cbSize = Marshal.SizeOf(deviceInterfaceData);
                    var deviceInterfaceIndex = 0;

                    while (NativeMethods.SetupDiEnumDeviceInterfaces(deviceInfoSet, ref deviceInfoData, ref hidClass, deviceInterfaceIndex, ref deviceInterfaceData))
                    {
                        deviceInterfaceIndex++;
                        var devicePath = GetDevicePath(deviceInfoSet, deviceInterfaceData);
                        var description = GetBusReportedDeviceDescription(deviceInfoSet, ref deviceInfoData) ??
                                          GetDeviceDescription(deviceInfoSet, ref deviceInfoData);
                        devices.Add(new DeviceInfo { Path = devicePath, Description = description });
                    }
                }
                NativeMethods.SetupDiDestroyDeviceInfoList(deviceInfoSet);
            }
            return devices;
        }

        private static NativeMethods.SP_DEVINFO_DATA CreateDeviceInfoData()
        {
            var deviceInfoData = new NativeMethods.SP_DEVINFO_DATA();

            deviceInfoData.cbSize = Marshal.SizeOf(deviceInfoData);
            deviceInfoData.DevInst = 0;
            deviceInfoData.ClassGuid = Guid.Empty;
            deviceInfoData.Reserved = IntPtr.Zero;

            return deviceInfoData;
        }

        private static string GetDevicePath(IntPtr deviceInfoSet, NativeMethods.SP_DEVICE_INTERFACE_DATA deviceInterfaceData)
        {
            var bufferSize = 0;
            var interfaceDetail = new NativeMethods.SP_DEVICE_INTERFACE_DETAIL_DATA { Size = IntPtr.Size == 4 ? 4 + Marshal.SystemDefaultCharSize : 8 };

            NativeMethods.SetupDiGetDeviceInterfaceDetailBuffer(deviceInfoSet, ref deviceInterfaceData, IntPtr.Zero, 0, ref bufferSize, IntPtr.Zero);

            return NativeMethods.SetupDiGetDeviceInterfaceDetail(deviceInfoSet, ref deviceInterfaceData, ref interfaceDetail, bufferSize, ref bufferSize, IntPtr.Zero) ?
                interfaceDetail.DevicePath : null;
        }

        private static Guid HidClassGuid
        {
            get
            {
                if (_hidClassGuid.Equals(Guid.Empty)) NativeMethods.HidD_GetHidGuid(ref _hidClassGuid);
                return _hidClassGuid;
            }
        }

        private static string GetDeviceDescription(IntPtr deviceInfoSet, ref NativeMethods.SP_DEVINFO_DATA devinfoData)
        {
            var descriptionBuffer = new byte[1024];

            var requiredSize = 0;
            var type = 0;

            NativeMethods.SetupDiGetDeviceRegistryProperty(deviceInfoSet,
                                                            ref devinfoData,
                                                            NativeMethods.SPDRP_DEVICEDESC,
                                                            ref type,
                                                            descriptionBuffer,
                                                            descriptionBuffer.Length,
                                                            ref requiredSize);

            return descriptionBuffer.ToUTF8String();
        }

        private static string GetBusReportedDeviceDescription(IntPtr deviceInfoSet, ref NativeMethods.SP_DEVINFO_DATA devinfoData)
        {
            var descriptionBuffer = new byte[1024];

            if (Environment.OSVersion.Version.Major > 5)
            {
                ulong propertyType = 0;
                var requiredSize = 0;

                var _continue = NativeMethods.SetupDiGetDeviceProperty(deviceInfoSet,
                                                                        ref devinfoData,
                                                                        ref NativeMethods.DEVPKEY_Device_BusReportedDeviceDesc,
                                                                        ref propertyType,
                                                                        descriptionBuffer,
                                                                        descriptionBuffer.Length,
                                                                        ref requiredSize,
                                                                        0);

                if (_continue) return descriptionBuffer.ToUTF16String();
            }
            return null;
        }
    }
    public static class Extensions
    {
        public static string ToUTF8String(this byte[] buffer)
        {
            var value = Encoding.UTF8.GetString(buffer);
            return value.Remove(value.IndexOf((char)0));
        }

        public static string ToUTF16String(this byte[] buffer)
        {
            var value = Encoding.Unicode.GetString(buffer);
            return value.Remove(value.IndexOf((char)0));
        }
    }
    internal static class NativeMethods
    {
        [StructLayout(LayoutKind.Sequential)]
        internal struct BLUETOOTH_FIND_RADIO_PARAMS
        {
            [MarshalAs(UnmanagedType.U4)]
            public int dwSize;
        }

        [DllImport("bthprops.cpl", CharSet = CharSet.Auto)]
        internal extern static IntPtr BluetoothFindFirstRadio(ref BLUETOOTH_FIND_RADIO_PARAMS pbtfrp, ref IntPtr phRadio);

        [DllImport("bthprops.cpl", CharSet = CharSet.Auto)]
        internal extern static bool BluetoothFindNextRadio(IntPtr hFind, ref IntPtr phRadio);

        [DllImport("bthprops.cpl", CharSet = CharSet.Auto)]
        internal extern static bool BluetoothFindRadioClose(IntPtr hFind);

        [DllImport("kernel32.dll", SetLastError = true)]
        internal static extern Boolean DeviceIoControl(IntPtr DeviceHandle, Int32 IoControlCode, ref long InBuffer, Int32 InBufferSize, IntPtr OutBuffer, Int32 OutBufferSize, ref Int32 BytesReturned, IntPtr Overlapped);

        [DllImport("kernel32.dll", SetLastError = true, ExactSpelling = true, CharSet = CharSet.Auto)]
        internal static extern bool CloseHandle(IntPtr hObject);

        internal const int FILE_FLAG_OVERLAPPED = 0x40000000;
        internal const short FILE_SHARE_READ = 0x1;
        internal const short FILE_SHARE_WRITE = 0x2;
        internal const uint GENERIC_READ = 0x80000000;
        internal const uint GENERIC_WRITE = 0x40000000;
        internal const Int32 FileShareRead = 1;
        internal const Int32 FileShareWrite = 2;
        internal const Int32 OpenExisting = 3;
        internal const int ACCESS_NONE = 0;
        internal const int INVALID_HANDLE_VALUE = -1;
        internal const short OPEN_EXISTING = 3;
        internal const int WAIT_TIMEOUT = 0x102;
        internal const uint WAIT_OBJECT_0 = 0;
        internal const uint WAIT_FAILED = 0xffffffff;

        internal const int WAIT_INFINITE = 0xffff;
        [StructLayout(LayoutKind.Sequential)]
        internal struct OVERLAPPED
        {
            public int Internal;
            public int InternalHigh;
            public int Offset;
            public int OffsetHigh;
            public int hEvent;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct SECURITY_ATTRIBUTES
        {
            public int nLength;
            public IntPtr lpSecurityDescriptor;
            public bool bInheritHandle;
        }

        [DllImport("kernel32.dll", SetLastError = true, ExactSpelling = true, CharSet = CharSet.Auto)]
        static internal extern bool CancelIo(IntPtr hFile);

        [DllImport("kernel32.dll", SetLastError = true, ExactSpelling = true, CharSet = CharSet.Auto)]
        static internal extern bool CancelIoEx(IntPtr hFile, IntPtr lpOverlapped);

        [DllImport("kernel32.dll", SetLastError = true, ExactSpelling = true, CharSet = CharSet.Auto)]
        static internal extern bool CancelSynchronousIo(IntPtr hObject);

        [DllImport("kernel32.dll", CharSet = CharSet.Auto)]
        static internal extern IntPtr CreateEvent(ref SECURITY_ATTRIBUTES securityAttributes, int bManualReset, int bInitialState, string lpName);

        [DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
        static internal extern IntPtr CreateFile(string lpFileName, uint dwDesiredAccess, int dwShareMode, ref SECURITY_ATTRIBUTES lpSecurityAttributes, int dwCreationDisposition, uint dwFlagsAndAttributes, int hTemplateFile);

        [DllImport("kernel32.dll", CharSet = CharSet.Auto, SetLastError = true)]
        internal static extern SafeFileHandle CreateFile(String lpFileName, UInt32 dwDesiredAccess, Int32 dwShareMode, IntPtr lpSecurityAttributes, Int32 dwCreationDisposition, UInt32 dwFlagsAndAttributes, Int32 hTemplateFile);
        [DllImport("kernel32.dll", SetLastError = true)]
        static internal extern bool ReadFile(IntPtr hFile, [Out] byte[] lpBuffer, uint nNumberOfBytesToRead, out uint lpNumberOfBytesRead, IntPtr lpOverlapped);

        [DllImport("kernel32.dll")]
        static internal extern uint WaitForSingleObject(IntPtr hHandle, int dwMilliseconds);

        [DllImport("kernel32.dll")]
        static internal extern bool WriteFile(IntPtr hFile, byte[] lpBuffer, uint nNumberOfBytesToWrite, out uint lpNumberOfBytesWritten, [In] ref System.Threading.NativeOverlapped lpOverlapped);

        internal const int DBT_DEVICEARRIVAL = 0x8000;
        internal const int DBT_DEVICEREMOVECOMPLETE = 0x8004;
        internal const int DBT_DEVTYP_DEVICEINTERFACE = 5;
        internal const int DBT_DEVTYP_HANDLE = 6;
        internal const int DEVICE_NOTIFY_ALL_INTERFACE_CLASSES = 4;
        internal const int DEVICE_NOTIFY_SERVICE_HANDLE = 1;
        internal const int DEVICE_NOTIFY_WINDOW_HANDLE = 0;
        internal const int WM_DEVICECHANGE = 0x219;
        internal const short DIGCF_PRESENT = 0x2;
        internal const short DIGCF_DEVICEINTERFACE = 0x10;
        internal const int DIGCF_ALLCLASSES = 0x4;
        internal const int DICS_ENABLE = 1;
        internal const int DICS_DISABLE = 2;
        internal const int DICS_FLAG_GLOBAL = 1;
        internal const int DIF_PROPERTYCHANGE = 0x12;

        internal const int MAX_DEV_LEN = 1000;
        internal const int SPDRP_ADDRESS = 0x1c;
        internal const int SPDRP_BUSNUMBER = 0x15;
        internal const int SPDRP_BUSTYPEGUID = 0x13;
        internal const int SPDRP_CAPABILITIES = 0xf;
        internal const int SPDRP_CHARACTERISTICS = 0x1b;
        internal const int SPDRP_CLASS = 7;
        internal const int SPDRP_CLASSGUID = 8;
        internal const int SPDRP_COMPATIBLEIDS = 2;
        internal const int SPDRP_CONFIGFLAGS = 0xa;
        internal const int SPDRP_DEVICE_POWER_DATA = 0x1e;
        internal const int SPDRP_DEVICEDESC = 0;
        internal const int SPDRP_DEVTYPE = 0x19;
        internal const int SPDRP_DRIVER = 9;
        internal const int SPDRP_ENUMERATOR_NAME = 0x16;
        internal const int SPDRP_EXCLUSIVE = 0x1a;
        internal const int SPDRP_FRIENDLYNAME = 0xc;
        internal const int SPDRP_HARDWAREID = 1;
        internal const int SPDRP_LEGACYBUSTYPE = 0x14;
        internal const int SPDRP_LOCATION_INFORMATION = 0xd;
        internal const int SPDRP_LOWERFILTERS = 0x12;
        internal const int SPDRP_MFG = 0xb;
        internal const int SPDRP_PHYSICAL_DEVICE_OBJECT_NAME = 0xe;
        internal const int SPDRP_REMOVAL_POLICY = 0x1f;
        internal const int SPDRP_REMOVAL_POLICY_HW_DEFAULT = 0x20;
        internal const int SPDRP_REMOVAL_POLICY_OVERRIDE = 0x21;
        internal const int SPDRP_SECURITY = 0x17;
        internal const int SPDRP_SECURITY_SDS = 0x18;
        internal const int SPDRP_SERVICE = 4;
        internal const int SPDRP_UI_NUMBER = 0x10;
        internal const int SPDRP_UI_NUMBER_DESC_FORMAT = 0x1d;

        internal const int SPDRP_UPPERFILTERS = 0x11;

        [StructLayout(LayoutKind.Sequential)]
        internal class DEV_BROADCAST_DEVICEINTERFACE
        {
            internal int dbcc_size;
            internal int dbcc_devicetype;
            internal int dbcc_reserved;
            internal Guid dbcc_classguid;
            internal short dbcc_name;
        }

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
        internal class DEV_BROADCAST_DEVICEINTERFACE_1
        {
            internal int dbcc_size;
            internal int dbcc_devicetype;
            internal int dbcc_reserved;
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.U1, SizeConst = 16)]
            internal byte[] dbcc_classguid;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 255)]
            internal char[] dbcc_name;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal class DEV_BROADCAST_HANDLE
        {
            internal int dbch_size;
            internal int dbch_devicetype;
            internal int dbch_reserved;
            internal int dbch_handle;
            internal int dbch_hdevnotify;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal class DEV_BROADCAST_HDR
        {
            internal int dbch_size;
            internal int dbch_devicetype;
            internal int dbch_reserved;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct SP_DEVICE_INTERFACE_DATA
        {
            internal int cbSize;
            internal System.Guid InterfaceClassGuid;
            internal int Flags;
            internal IntPtr Reserved;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct SP_DEVINFO_DATA
        {
            internal int cbSize;
            internal Guid ClassGuid;
            internal int DevInst;
            internal IntPtr Reserved;
        }

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Auto)]
        internal struct SP_DEVICE_INTERFACE_DETAIL_DATA
        {
            internal int Size;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
            internal string DevicePath;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct DEVPROPKEY
        {
            public Guid fmtid;
            public ulong pid;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct SP_CLASSINSTALL_HEADER
        {
            internal int cbSize;
            internal int installFunction;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct SP_PROPCHANGE_PARAMS
        {
            internal SP_CLASSINSTALL_HEADER classInstallHeader;
            internal int stateChange;
            internal int scope;
            internal int hwProfile;
        }

        internal static DEVPROPKEY DEVPKEY_Device_BusReportedDeviceDesc =
            new DEVPROPKEY { fmtid = new Guid(0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2), pid = 4 };

        internal static DEVPROPKEY DEVPKEY_Device_HardwareIds =
            new DEVPROPKEY { fmtid = new Guid(0xa45c254e, 0xdf1c, 0x4efd, 0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0), pid = 3 };

        [DllImport("setupapi.dll", EntryPoint = "SetupDiGetDeviceRegistryProperty")]
        public static extern bool SetupDiGetDeviceRegistryProperty(IntPtr deviceInfoSet, ref SP_DEVINFO_DATA deviceInfoData, int propertyVal, ref int propertyRegDataType, byte[] propertyBuffer, int propertyBufferSize, ref int requiredSize);

        [DllImport("setupapi.dll", EntryPoint = "SetupDiGetDevicePropertyW", SetLastError = true)]
        public static extern bool SetupDiGetDeviceProperty(IntPtr deviceInfo, ref SP_DEVINFO_DATA deviceInfoData, ref DEVPROPKEY propkey, ref ulong propertyDataType, byte[] propertyBuffer, int propertyBufferSize, ref int requiredSize, uint flags);

        [DllImport("setupapi.dll")]
        static internal extern bool SetupDiEnumDeviceInfo(IntPtr deviceInfoSet, int memberIndex, ref SP_DEVINFO_DATA deviceInfoData);

        [DllImport("user32.dll", CharSet = CharSet.Auto)]
        static internal extern IntPtr RegisterDeviceNotification(IntPtr hRecipient, IntPtr notificationFilter, Int32 flags);

        [DllImport("setupapi.dll")]
        internal static extern int SetupDiCreateDeviceInfoList(ref Guid classGuid, int hwndParent);

        [DllImport("setupapi.dll")]
        static internal extern int SetupDiDestroyDeviceInfoList(IntPtr deviceInfoSet);

        [DllImport("setupapi.dll")]
        static internal extern bool SetupDiEnumDeviceInterfaces(IntPtr deviceInfoSet, ref SP_DEVINFO_DATA deviceInfoData, ref Guid interfaceClassGuid, int memberIndex, ref SP_DEVICE_INTERFACE_DATA deviceInterfaceData);

        [DllImport("setupapi.dll", CharSet = CharSet.Auto)]
        static internal extern IntPtr SetupDiGetClassDevs(ref System.Guid classGuid, string enumerator, int hwndParent, int flags);

        [DllImport("setupapi.dll", CharSet = CharSet.Auto, EntryPoint = "SetupDiGetDeviceInterfaceDetail")]
        static internal extern bool SetupDiGetDeviceInterfaceDetailBuffer(IntPtr deviceInfoSet, ref SP_DEVICE_INTERFACE_DATA deviceInterfaceData, IntPtr deviceInterfaceDetailData, int deviceInterfaceDetailDataSize, ref int requiredSize, IntPtr deviceInfoData);

        [DllImport("setupapi.dll", CharSet = CharSet.Auto)]
        static internal extern bool SetupDiGetDeviceInterfaceDetail(IntPtr deviceInfoSet, ref SP_DEVICE_INTERFACE_DATA deviceInterfaceData, ref SP_DEVICE_INTERFACE_DETAIL_DATA deviceInterfaceDetailData, int deviceInterfaceDetailDataSize, ref int requiredSize, IntPtr deviceInfoData);

        [DllImport("setupapi.dll", CharSet = CharSet.Auto)]
        static internal extern bool SetupDiSetClassInstallParams(IntPtr deviceInfoSet, ref SP_DEVINFO_DATA deviceInfoData, ref SP_PROPCHANGE_PARAMS classInstallParams, int classInstallParamsSize);

        [DllImport("setupapi.dll", CharSet = CharSet.Auto)]
        static internal extern bool SetupDiCallClassInstaller(int installFunction, IntPtr deviceInfoSet, ref SP_DEVINFO_DATA deviceInfoData);

        [DllImport("setupapi.dll", CharSet = CharSet.Auto)]
        static internal extern bool SetupDiGetDeviceInstanceId(IntPtr deviceInfoSet, ref SP_DEVINFO_DATA deviceInfoData, char[] deviceInstanceId, Int32 deviceInstanceIdSize, ref int requiredSize);

        [DllImport("user32.dll")]
        static internal extern bool UnregisterDeviceNotification(IntPtr handle);

        internal const short HIDP_INPUT = 0;
        internal const short HIDP_OUTPUT = 1;

        internal const short HIDP_FEATURE = 2;
        [StructLayout(LayoutKind.Sequential)]
        internal struct HIDD_ATTRIBUTES
        {
            internal int Size;
            internal ushort VendorID;
            internal ushort ProductID;
            internal short VersionNumber;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct HIDP_CAPS
        {
            internal short Usage;
            internal short UsagePage;
            internal short InputReportByteLength;
            internal short OutputReportByteLength;
            internal short FeatureReportByteLength;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 17)]
            internal short[] Reserved;
            internal short NumberLinkCollectionNodes;
            internal short NumberInputButtonCaps;
            internal short NumberInputValueCaps;
            internal short NumberInputDataIndices;
            internal short NumberOutputButtonCaps;
            internal short NumberOutputValueCaps;
            internal short NumberOutputDataIndices;
            internal short NumberFeatureButtonCaps;
            internal short NumberFeatureValueCaps;
            internal short NumberFeatureDataIndices;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct HIDP_VALUE_CAPS
        {
            internal short UsagePage;
            internal byte ReportID;
            internal int IsAlias;
            internal short BitField;
            internal short LinkCollection;
            internal short LinkUsage;
            internal short LinkUsagePage;
            internal int IsRange;
            internal int IsStringRange;
            internal int IsDesignatorRange;
            internal int IsAbsolute;
            internal int HasNull;
            internal byte Reserved;
            internal short BitSize;
            internal short ReportCount;
            internal short Reserved2;
            internal short Reserved3;
            internal short Reserved4;
            internal short Reserved5;
            internal short Reserved6;
            internal int LogicalMin;
            internal int LogicalMax;
            internal int PhysicalMin;
            internal int PhysicalMax;
            internal short UsageMin;
            internal short UsageMax;
            internal short StringMin;
            internal short StringMax;
            internal short DesignatorMin;
            internal short DesignatorMax;
            internal short DataIndexMin;
            internal short DataIndexMax;
        }

        [DllImport("hid.dll")]
        static internal extern bool HidD_FlushQueue(IntPtr hidDeviceObject);

        [DllImport("hid.dll")]
        static internal extern bool HidD_FlushQueue(SafeFileHandle hidDeviceObject);

        [DllImport("hid.dll")]
        static internal extern bool HidD_GetAttributes(IntPtr hidDeviceObject, ref HIDD_ATTRIBUTES attributes);

        [DllImport("hid.dll")]
        static internal extern bool HidD_GetFeature(IntPtr hidDeviceObject, byte[] lpReportBuffer, int reportBufferLength);

        [DllImport("hid.dll", SetLastError = true)]
        internal static extern Boolean HidD_GetInputReport(SafeFileHandle HidDeviceObject, Byte[] lpReportBuffer, Int32 ReportBufferLength);

        [DllImport("hid.dll")]
        static internal extern void HidD_GetHidGuid(ref Guid hidGuid);

        [DllImport("hid.dll")]
        static internal extern bool HidD_GetNumInputBuffers(IntPtr hidDeviceObject, ref int numberBuffers);

        [DllImport("hid.dll")]
        static internal extern bool HidD_GetPreparsedData(IntPtr hidDeviceObject, ref IntPtr preparsedData);

        [DllImport("hid.dll")]
        static internal extern bool HidD_FreePreparsedData(IntPtr preparsedData);

        [DllImport("hid.dll")]
        static internal extern bool HidD_SetFeature(IntPtr hidDeviceObject, byte[] lpReportBuffer, int reportBufferLength);

        [DllImport("hid.dll")]
        static internal extern bool HidD_SetFeature(SafeFileHandle hidDeviceObject, byte[] lpReportBuffer, int reportBufferLength);

        [DllImport("hid.dll")]
        static internal extern bool HidD_SetNumInputBuffers(IntPtr hidDeviceObject, int numberBuffers);

        [DllImport("hid.dll")]
        static internal extern bool HidD_SetOutputReport(IntPtr hidDeviceObject, byte[] lpReportBuffer, int reportBufferLength);

        [DllImport("hid.dll", SetLastError = true)]
        static internal extern bool HidD_SetOutputReport(SafeFileHandle hidDeviceObject, byte[] lpReportBuffer, int reportBufferLength);

        [DllImport("hid.dll")]
        static internal extern int HidP_GetCaps(IntPtr preparsedData, ref HIDP_CAPS capabilities);

        [DllImport("hid.dll")]
        static internal extern int HidP_GetValueCaps(short reportType, ref byte valueCaps, ref short valueCapsLength, IntPtr preparsedData);

#if WIN64
        [DllImport("hid.dll")]
        static internal extern bool HidD_GetSerialNumberString(IntPtr HidDeviceObject, byte[] Buffer, ulong BufferLength);
#else
        [DllImport("hid.dll")]
        static internal extern bool HidD_GetSerialNumberString(IntPtr HidDeviceObject, byte[] Buffer, uint BufferLength);
#endif
    }*/
}
