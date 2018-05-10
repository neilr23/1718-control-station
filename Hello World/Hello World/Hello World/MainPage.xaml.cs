using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using Windows.Gaming.Input;
using System.Threading;

// The Blank Page item template is documented at https://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace Hello_World
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        private String myKey;
        private RawGameController pilotOneControl;
        private RawGameController pilotTwoControl;
        private Timer pilotTimer;
        private int buttonCount;
        private int axisCount;
        private Boolean[] buttonStates;
        private Double[] axisStates;
        private Boolean controllersOnline;
        private int currentController;
        public MainPage()
        {
            this.InitializeComponent();
            myKey = "";
            AutoResetEvent AutoEvent = new AutoResetEvent(true);
            pilotTimer = new Timer(new TimerCallback(checkStatus), AutoEvent, 0, 1);
            currentController = 0; 
        }

        private void HorizontalButton_Click(object sender, RoutedEventArgs e)
        {

        }
        private void HippityHoppityYoureMyProperty(object sender, RoutedEventArgs e)
        {
            PilotOne.Text = myKey;
        }
        void MainPage_KeyDown(object sender, KeyRoutedEventArgs e)
        {
            myKey = e.Key + "";
            HippityHoppityYoureMyProperty(this, new RoutedEventArgs());
        }
        private void checkStatus(object state)
        {
            if(controllersOnline)
            {

            }
        }
    }
}
