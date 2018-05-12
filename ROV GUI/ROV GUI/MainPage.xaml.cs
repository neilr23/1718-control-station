
using System;
using System.Threading;
using System.Collections;
using System.Threading.Tasks;
using System.Linq;
using System.Runtime.InteropServices;
using System.Security.Principal;
using System.Security.Cryptography;

using Windows.Devices.SerialCommunication;
using Windows.Devices.Enumeration;
using System.Collections.Generic;
using System.Diagnostics;

using Windows.Storage.Streams;

using Windows.Gaming.Input;

using Windows.UI.Core;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Media.Imaging;





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
            ConnectedControllers.Text = RawGameController.RawGameControllers + "";
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
                         ControllerTester.Text = buttonOneStates[0] + "";
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
    //this bullshit is from DS4
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

        ///DS4DEVICES
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
                /*if (!success)
                {
                    throw new Exception("Error disabling device, error code = " + Marshal.GetLastWin32Error());
                }
                */

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
                public class DS4StateExposed
            {
                private DS4State _state;

                public DS4StateExposed()
                {
                    _state = new DS4State();
                }

                public DS4StateExposed(DS4State state)
                {
                    _state = state;
                }

                bool Square { get { return _state.Square; } }
                bool Triangle { get { return _state.Triangle; } }
                bool Circle { get { return _state.Circle; } }
                bool Cross { get { return _state.Cross; } }
                bool DpadUp { get { return _state.DpadUp; } }
                bool DpadDown { get { return _state.DpadDown; } }
                bool DpadLeft { get { return _state.DpadLeft; } }
                bool DpadRight { get { return _state.DpadRight; } }
                bool L1 { get { return _state.L1; } }
                bool L3 { get { return _state.L3; } }
                bool R1 { get { return _state.R1; } }
                bool R3 { get { return _state.R3; } }
                bool Share { get { return _state.Share; } }
                bool Options { get { return _state.Options; } }
                bool PS { get { return _state.PS; } }
                bool Touch1 { get { return _state.Touch1; } }
                bool Touch2 { get { return _state.Touch2; } }
                bool TouchButton { get { return _state.TouchButton; } }
                bool Touch1Finger { get { return _state.Touch1Finger; } }
                bool Touch2Fingers { get { return _state.Touch2Fingers; } }
                byte LX { get { return _state.LX; } }
                byte RX { get { return _state.RX; } }
                byte LY { get { return _state.LY; } }
                byte RY { get { return _state.RY; } }
                byte L2 { get { return _state.L2; } }
                byte R2 { get { return _state.R2; } }
                int Battery { get { return _state.Battery; } }

                public int GyroYaw { get { return _state.Motion.gyroYaw; } }
                public int getGyroYaw()
                {
                    return _state.Motion.gyroYaw;
                }

                public int GyroPitch { get { return _state.Motion.gyroPitch; } }
                public int getGyroPitch()
                {
                    return _state.Motion.gyroPitch;
                }

                public int GyroRoll { get { return _state.Motion.gyroRoll; } }
                public int getGyroRoll()
                {
                    return _state.Motion.gyroRoll;
                }

                public int AccelX { get { return _state.Motion.accelX; } }
                public int getAccelX()
                {
                    return _state.Motion.accelX;
                }

                public int AccelY { get { return _state.Motion.accelY; } }
                public int getAccelY()
                {
                    return _state.Motion.accelY;
                }

                public int AccelZ { get { return _state.Motion.accelZ; } }
                public int getAccelZ()
                {
                    return _state.Motion.accelZ;
                }

                public int OutputAccelX { get { return _state.Motion.outputAccelX; } }
                public int getOutputAccelX()
                {
                    return _state.Motion.outputAccelX;
                }

                public int OutputAccelY { get { return _state.Motion.outputAccelY; } }
                public int getOutputAccelY()
                {
                    return _state.Motion.outputAccelY;
                }

                public int OutputAccelZ { get { return _state.Motion.outputAccelZ; } }
                public int getOutputAccelZ()
                {
                    return _state.Motion.outputAccelZ;
                }
            }
        }
        }
    }
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
//DS4SIXAXIS
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
    public class TouchpadEventArgs : EventArgs
    {
        public readonly Touch[] touches = null;
        public readonly DateTime timeStamp;
        public readonly bool touchButtonPressed;
        public TouchpadEventArgs(DateTime utcTimestamp, bool tButtonDown, Touch t0, Touch t1 = null)
        {
            if (t1 != null)
            {
                touches = new Touch[2];
                touches[0] = t0;
                touches[1] = t1;
            }
            else if (t0 != null)
            {
                touches = new Touch[1];
                touches[0] = t0;
            }

            touchButtonPressed = tButtonDown;
            timeStamp = utcTimestamp;
        }
    }

    public class Touch
    {
        public int hwX, hwY, deltaX, deltaY;
        public byte touchID;
        public Touch previousTouch;
        internal Touch(int X, int Y, byte tID, Touch prevTouch = null)
        {
            populate(X, Y, tID, prevTouch);
        }

        internal void populate(int X, int Y, byte tID, Touch prevTouch = null)
        {
            hwX = X;
            hwY = Y;
            touchID = tID;
            previousTouch = prevTouch;
            if (previousTouch != null)
            {
                deltaX = X - previousTouch.hwX;
                deltaY = Y - previousTouch.hwY;
            }
        }
    }

    public class DS4Touchpad
    {
        public event EventHandler<TouchpadEventArgs> TouchesBegan = null; // finger one or two landed (or both, or one then two, or two then one; any touches[] count increase)
        public event EventHandler<TouchpadEventArgs> TouchesMoved = null; // deltaX/deltaY are set because one or both fingers were already down on a prior sensor reading
        public event EventHandler<TouchpadEventArgs> TouchesEnded = null; // all fingers lifted
        public event EventHandler<TouchpadEventArgs> TouchButtonDown = null; // touchpad pushed down until the button clicks
        public event EventHandler<TouchpadEventArgs> TouchButtonUp = null; // touchpad button released
        public event EventHandler<EventArgs> TouchUnchanged = null; // no status change for the touchpad itself... but other sensors may have changed, or you may just want to do some processing
        public event EventHandler<EventArgs> PreTouchProcess = null; // used to publish that a touch packet is about to be processed

        public readonly static int TOUCHPAD_DATA_OFFSET = 35;
        internal int lastTouchPadX1, lastTouchPadY1,
            lastTouchPadX2, lastTouchPadY2; // tracks 0, 1 or 2 touches; we maintain touch 1 and 2 separately
        internal bool lastTouchPadIsDown;
        internal bool lastIsActive1, lastIsActive2;
        internal byte lastTouchID1, lastTouchID2;
        internal byte[] previousPacket = new byte[8];
        private Touch tPrev0, tPrev1, t0, t1;

        public DS4Touchpad()
        {
            tPrev0 = new Touch(0, 0, 0);
            tPrev1 = new Touch(0, 0, 0);
            t0 = new Touch(0, 0, 0);
            t1 = new Touch(0, 0, 0);
        }

        // We check everything other than the not bothering with not-very-useful TouchPacketCounter.
        private bool PacketChanged(byte[] data, int touchPacketOffset)
        {
            bool changed = false;
            for (int i = 0, arLen = previousPacket.Length; !changed && i < arLen; i++)
            {
                //byte oldValue = previousPacket[i];
                //previousPacket[i] = data[i + TOUCHPAD_DATA_OFFSET + touchPacketOffset];
                if (previousPacket[i] != data[i + TOUCHPAD_DATA_OFFSET + touchPacketOffset])
                    changed = true;
            }

            return changed;
        }

        public void handleTouchpad(byte[] data, DS4State sensors, int touchPacketOffset = 0)
        {
            PreTouchProcess?.Invoke(this, EventArgs.Empty);

            bool touchPadIsDown = sensors.TouchButton;
            if (!PacketChanged(data, touchPacketOffset) && touchPadIsDown == lastTouchPadIsDown)
            {
                if (TouchUnchanged != null)
                    TouchUnchanged(this, EventArgs.Empty);
                return;
            }

            Array.Copy(data, TOUCHPAD_DATA_OFFSET + touchPacketOffset, previousPacket, 0, 8);
            byte touchID1 = (byte)(data[0 + TOUCHPAD_DATA_OFFSET + touchPacketOffset] & 0x7F);
            byte touchID2 = (byte)(data[4 + TOUCHPAD_DATA_OFFSET + touchPacketOffset] & 0x7F);
            int currentX1 = ((data[2 + TOUCHPAD_DATA_OFFSET + touchPacketOffset] & 0x0F) << 8) | data[1 + TOUCHPAD_DATA_OFFSET + touchPacketOffset];
            int currentY1 = (data[3 + TOUCHPAD_DATA_OFFSET + touchPacketOffset] << 4) | ((data[2 + TOUCHPAD_DATA_OFFSET + touchPacketOffset] & 0xF0) >> 4);
            int currentX2 = ((data[6 + TOUCHPAD_DATA_OFFSET + touchPacketOffset] & 0x0F) << 8) | data[5 + TOUCHPAD_DATA_OFFSET + touchPacketOffset];
            int currentY2 = (data[7 + TOUCHPAD_DATA_OFFSET + touchPacketOffset] << 4) | ((data[6 + TOUCHPAD_DATA_OFFSET + touchPacketOffset] & 0xF0) >> 4);

            TouchpadEventArgs args;
            if (sensors.Touch1 || sensors.Touch2)
            {
                if ((sensors.Touch1 && !lastIsActive1) || (sensors.Touch2 && !lastIsActive2))
                {
                    if (TouchesBegan != null)
                    {
                        if (sensors.Touch1 && sensors.Touch2)
                        {
                            t0.populate(currentX1, currentY1, touchID1); t1.populate(currentX2, currentY2, touchID2);
                            args = new TouchpadEventArgs(sensors.ReportTimeStamp, sensors.TouchButton, t0, t1);
                        }
                        else if (sensors.Touch1)
                        {
                            t0.populate(currentX1, currentY1, touchID1);
                            args = new TouchpadEventArgs(sensors.ReportTimeStamp, sensors.TouchButton, t0);
                        }
                        else
                        {
                            t0.populate(currentX2, currentY2, touchID2);
                            args = new TouchpadEventArgs(sensors.ReportTimeStamp, sensors.TouchButton, t0);
                        }

                        TouchesBegan(this, args);
                    }
                }
                else if (sensors.Touch1 == lastIsActive1 && sensors.Touch2 == lastIsActive2 && TouchesMoved != null)
                {
                    Touch currentT0, currentT1;

                    if (sensors.Touch1 && sensors.Touch2)
                    {
                        tPrev0.populate(lastTouchPadX1, lastTouchPadY1, lastTouchID1);
                        t0.populate(currentX1, currentY1, touchID1, tPrev0);
                        currentT0 = t0;

                        tPrev1.populate(lastTouchPadX2, lastTouchPadY2, lastTouchID2);
                        t1.populate(currentX2, currentY2, touchID2, tPrev1);
                        currentT1 = t1;
                    }
                    else if (sensors.Touch1)
                    {
                        tPrev0.populate(lastTouchPadX1, lastTouchPadY1, lastTouchID1);
                        t0.populate(currentX1, currentY1, touchID1, tPrev0);
                        currentT0 = t0;
                        currentT1 = null;
                    }
                    else
                    {
                        tPrev0.populate(lastTouchPadX2, lastTouchPadY2, lastTouchID2);
                        t0.populate(currentX2, currentY2, touchID2, tPrev0);
                        currentT0 = t0;
                        currentT1 = null;
                    }

                    args = new TouchpadEventArgs(sensors.ReportTimeStamp, sensors.TouchButton, currentT0, currentT1);

                    TouchesMoved(this, args);
                }

                if (!lastTouchPadIsDown && touchPadIsDown && TouchButtonDown != null)
                {
                    if (sensors.Touch1 && sensors.Touch2)
                    {
                        t0.populate(currentX1, currentY1, touchID1);
                        t1.populate(currentX2, currentY2, touchID2);
                        args = new TouchpadEventArgs(sensors.ReportTimeStamp, sensors.TouchButton, t0, t1);
                    }
                    else if (sensors.Touch1)
                    {
                        t0.populate(currentX1, currentY1, touchID1);
                        args = new TouchpadEventArgs(sensors.ReportTimeStamp, sensors.TouchButton, t0);
                    }
                    else
                    {
                        t0.populate(currentX2, currentY2, touchID2);
                        args = new TouchpadEventArgs(sensors.ReportTimeStamp, sensors.TouchButton, t0);
                    }

                    TouchButtonDown(this, args);
                }
                else if (lastTouchPadIsDown && !touchPadIsDown && TouchButtonUp != null)
                {
                    if (sensors.Touch1 && sensors.Touch2)
                    {
                        t0.populate(currentX1, currentY1, touchID1);
                        t1.populate(currentX2, currentY2, touchID2);
                        args = new TouchpadEventArgs(sensors.ReportTimeStamp, sensors.TouchButton, t0, t1);
                    }
                    else if (sensors.Touch1)
                    {
                        t0.populate(currentX1, currentY1, touchID1);
                        args = new TouchpadEventArgs(sensors.ReportTimeStamp, sensors.TouchButton, t0);
                    }
                    else
                    {
                        t0.populate(currentX2, currentY2, touchID2);
                        args = new TouchpadEventArgs(sensors.ReportTimeStamp, sensors.TouchButton, t0);
                    }

                    TouchButtonUp(this, args);
                }

                if (sensors.Touch1)
                {
                    lastTouchPadX1 = currentX1;
                    lastTouchPadY1 = currentY1;
                }
                if (sensors.Touch2)
                {
                    lastTouchPadX2 = currentX2;
                    lastTouchPadY2 = currentY2;
                }

                lastTouchPadIsDown = touchPadIsDown;
            }
            else
            {
                if (touchPadIsDown && !lastTouchPadIsDown)
                {
                    if (TouchButtonDown != null)
                        TouchButtonDown(this, new TouchpadEventArgs(sensors.ReportTimeStamp, sensors.TouchButton, null, null));
                }
                else if (!touchPadIsDown && lastTouchPadIsDown)
                {
                    if (TouchButtonUp != null)
                        TouchButtonUp(this, new TouchpadEventArgs(sensors.ReportTimeStamp, sensors.TouchButton, null, null));
                }

                if ((lastIsActive1 || lastIsActive2) && TouchesEnded != null)
                {
                    if (lastIsActive1 && lastIsActive2)
                    {
                        t0.populate(lastTouchPadX1, lastTouchPadY1, touchID1);
                        t1.populate(lastTouchPadX2, lastTouchPadY2, touchID2);
                        args = new TouchpadEventArgs(sensors.ReportTimeStamp, sensors.TouchButton, t0, t1);
                    }
                    else if (lastIsActive1)
                    {
                        t0.populate(lastTouchPadX1, lastTouchPadY1, touchID1);
                        args = new TouchpadEventArgs(sensors.ReportTimeStamp, sensors.TouchButton, t0);
                    }
                    else
                    {
                        t0.populate(lastTouchPadX2, lastTouchPadY2, touchID2);
                        args = new TouchpadEventArgs(sensors.ReportTimeStamp, sensors.TouchButton, t0);
                    }

                    TouchesEnded(this, args);
                }
            }

            lastIsActive1 = sensors.Touch1;
            lastIsActive2 = sensors.Touch2;
            lastTouchID1 = touchID1;
            lastTouchID2 = touchID2;
            lastTouchPadIsDown = touchPadIsDown;
        }
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
                /*uint crc = seed;
                for (int i = start; i < size + start; i++)
                    crc = (crc >> 8) ^ defaultTable[buffer[i] ^ crc & 0xff];
                return crc;
                */

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
                    /*crc ^= buffer[i++] |
                                (uint)(buffer[i++] << 8) |
                                (uint)(buffer[i++] << 16) |
                                (uint)(buffer[i++] << 24);// i = i + 4;
                    //crc ^= buffer[i];
                    crc = secondLook[3, (crc & 0xFF)] ^
                        secondLook[2, ((crc >> 8) & 0xFF)] ^
                        secondLook[1, ((crc >> 16) & 0xFF)] ^
                        defaultTable[crc >> 24];
                    bufsize -= 4;
                    */


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
    }
}
