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
using System.Timers.Timer; 
using System.Threading;

// The Blank Page item template is documented at https://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace Hello_World
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        public MainPage()
        {
            this.InitializeComponent();
            GetSticks();
            Sticks = GetSticks();
            timer1.Enabled = false;
        }
        DirectInput Input = new DirectInput();
        SlimDX.DirectInput.Joystick stick;
        Joystick[] Sticks;
        bool MouseClicked = false;

        int yValue = 0;
        int xValue = 0;
        int zValue = 0;

        [DllImport("user32.dll", CharSet = CharSet.Auto)]
        public static extern void mouse_event(uint flag, uint _x, uint _y, uint btn, uint exInfo);
        private const int MOUSEEVENT_LEFTDOWN = 0x02;
        private const int MOUSEEVENT_LEFTU = 0x04;
        public Joystick[] GetSticks()
        {
            List<SlimDX.DirectInput.Joystick> sticks = new List<SlimDX.DirectInput.Joystick>();
            foreach(DeviceInstance device in Input.GetDevices(DeviceVlass.GameController, DevicEnumeratinFlags.AttachedOnly))
            {
                try
                {
                    stick = new SlimDX.DirectInput.Joystick(Input, device.InstanceGuid);
                    stick.Acquire();
                    foreach(DeviceObjectInstance deviceObject in stick.GetObjects())
                    {
                        if(deviceObject.ObjectType & ObjectDeviceType.Axis != 0)
                        {
                            stick.GetObjectPropertiesById((int)deviceObject.ObjectType).SetRange(-100, 100);
                        }
                    }
                    sticks.Add(stick);
                }catch(DirectInputException)
                {

                }
            }
                return sticks.ToArray();
        }
        void stickHandle(Joystick stick, int id)
        {
            JoystickState state = new JoystickState();
            state = stick.GetCurrentState();

            xValue = state.X/3;
            yValue = state.Y/3;
            zValue = state.Z/3;
            MouseMove(xValue, yValue);
            bool[] buttons = state.GetButtons();

            if(id == 0)
            {
                if(buttons[0])
                {
                    if(mouseClicked == false)
                    {
                        mouse_event(MOUSEEVENT_LEFTDOWN, 0, 0, 0, 0);
                        mouseClicked = true;
                    }
                }
                else
                {
                    if(MouseClicked == true)
                    {
                        mouse_event(MOUSEEVENT_LEFTUP, 0, 0, 0, 0);
                        MouseClicked = false;
                    }

                }
            }
        }
        public void MouseMove(int posx, int posy)
        {
            Cursor.Position = new Point(Cursor.PositionX + posx, Cursor.PositionY + posy);
            Cursor.Clip = new RectangleGeometry(this.location, this.size);
        }
        private void timer1_Tick(object sender, EventArgs e)
        {
            for (int a = 0; a < Sticks.Length; a++)
            {
                stickHandle(Sticks[a], a);
            }
        }
        private void dispatcherTimer_Tick(object sender, EventArgs e)
        {
            // code goes here
        }
        private void TextBlock_SelectionChanged(object sender, RoutedEventArgs e)
        {

        }

        private void TextBlock_SelectionChanged_1(object sender, RoutedEventArgs e)
        {

        }

        private void TextBox_TextChanged(object sender, TextChangedEventArgs e)
        {

        }

        private void TextBox_TextChanged_1(object sender, TextChangedEventArgs e)
        {

        }

        private void AppBarButton_Click(object sender, RoutedEventArgs e)
        {

        }
        private void Form1_Load(object sender, EventArgs e)
        {
            Joystick[] joystick = GetSticks();
        }
    }
}
