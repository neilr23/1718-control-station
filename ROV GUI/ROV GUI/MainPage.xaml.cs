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
                         baudSpeedId.Text = "" + baudRate;
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
                         if (controller1)
                             Controller1Img.Source = new BitmapImage(new Uri("ms-appx:///Assets/ps4_controller1.jpg"));
                         else
                             Controller1Img.Source = new BitmapImage(new Uri("ms-appx:///Assets/Square44x44Logo.png"));
                         if (controller2)
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
                if (int.TryParse(baudInput.Text, out num))
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
        public static ArrayList RawGameControllers { get; set; }

        private void checkAndSend(object state)//We send things here
        {
            string deploy = "";
            try
            {
                if (!(RawGameControllers.Count == 0))
                {
                    if (!(RawGameControllers.Count == 0))
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
                        controllers[0].GetCurrentReading(buttonOneStates, null, axisOneStates);
                        controllers[1].GetCurrentReading(buttonTwoStates, null, axisTwoStates);
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
            }
            catch (Exception ex)
            {
                connected = false;
                ContentDialog failedtoConnectControllersDialog = new ContentDialog
                {
                    Title = "Boi where are the controllers?",
                    Content = ex.ToString(),
                    CloseButtonText = "OH SUGAR"
                };
                //ContentDialogResult result = await failedtoConnectControllersDialog.ShowAsync();
            }
                ContentDialogResult result = await failedtoConnectControllersDialog.ShowAsync();
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
    }
}
