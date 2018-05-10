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
        public MainPage()
        {
            this.InitializeComponent();
            myKey = "";
        };

        private void HorizontalButton_Click(object sender, RoutedEventArgs e)
        {

        }
        private void TippityTappityYoureMyProperty(object sender, RoutedEventArgs e)
        {
            PilotOne.Text = myKey;
        }
        void Address_KeyDown(object sender, KeyRoutedEventArgs e)
        {
            myKey = e.Key + "";
            TippityTappityYoureMyProperty(this, new RoutedEventArgs());
        }
    }
}
