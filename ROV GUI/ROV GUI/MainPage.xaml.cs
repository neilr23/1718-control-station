using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using System;
using System.Threading;
using System.Threading.Tasks;
using Windows.Devices.SerialCommunication;
using Windows.Devices.Enumeration;
using Windows.Storage.Streams;
using Windows.Gaming.Input;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml;


// The Blank Page item template is documented at https://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace ROV_GUI
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        //COMMs
        private SerialDevice UartPort;
        private DataReader DataReaderObject = null;
        private DataWriter DataWriterObject;
        private CancellationTokenSource ReadCancellationTokenSource;
        private byte[] modbusRegisters;
        private byte[] manipRegisters;
        private String myCom;
        //Controller
        private RawGameController[] controllers;
        private Timer pilotTimer;
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
            //Controller
            AutoResetEvent AutoEvent = new AutoResetEvent(true);
            pilotTimer = new Timer(new TimerCallback(checkAndSend), AutoEvent, 0, 1); //Timer to do things every fucking millisecond lmaooo
            controllers = new RawGameController[RawGameController.RawGameControllers.Count];
            for(int a = 0; a < RawGameController.RawGameControllers.Count; a++)
            {
                controllers[a] = RawGameController.RawGameControllers[a];
            }
            //COMMs
            string selector = SerialDevice.GetDeviceSelector("COM3");
            modbusRegisters = new byte[28];
            manipRegisters = new byte[4];
            myCom = "";
            Task t = Initialise(9600);
            t.Start();
        }
        /////////////////////////////////////////////////////////////////////////////GUI interactions
        private void HorizontalButton_Click(object sender, RoutedEventArgs e)
        {

        }

        private void Slider_ValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            myCom = "COM " + Slider.ValueProperty;
        }
        /////////////////////////////////////////////////////////////////////////////Backend
        private void checkAndSend(object state)//We send things here
        {
                string deploy = "";
                controllers[0].GetCurrentReading(buttonOneStates, null, axisOneStates);
                controllers[1].GetCurrentReading(buttonTwoStates, null, axisTwoStates);
                for(int a = 0; a < buttonOneStates.Length; a++)
                {
                    deploy += "Button " + a + ": " + buttonOneStates[a] + " |";
                }
                PilotOne.Text = deploy;
                //a whole shit ton of if statements of what happens when buttons are pressed, put results into modbusRegisters and manipRegisters
                //See the arduino code for explanations of what each place in modbusRegisters represents. 
            SendBytes(modbusRegisters, manipRegisters);
        }

        public async Task Initialise(uint BaudRate)     //NOTE - THIS IS AN ASYNC METHOD!
        {
            try
            {
                string aqs = SerialDevice.GetDeviceSelector("UART0");
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

                StartReceive();
            }
            catch (Exception ex)
            {
                throw new Exception("Uart Initialise Error", ex);
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
                throw new Exception("Uart Tx Error", ex);
            }
        }
    }
}
