
#region 3D private Ojw.C3d m_C3d = new Ojw.C3d (); 
private void Form1_Load (object sender, EventArgs e) 
{ 
    // specifies to use message box 
    Ojw.CMessage.Init (txtMessage);

    // Use 3d Definition 
    m_C3d.Init (picDisp); 
            
    // Define property window 
    m_C3d.CreateProp_VirtualObject (picProperty); 
    if (m_C3d.FileOpen (" Joystick.ojw ") == true) 
    { 
        timer1.Enabled = true; 
    } 
} 
#endregion 3D

private Ojw.CJoystick m_CJoy = new Ojw.CJoystick (Ojw.CJoystick._ID_0); // Joystick declaration

private Ojw.CTimer m_CTmr_Joystick = new Ojw.CTimer (); // Timer to periodically check joystick connections

private float [] m_afAngle = new float [20]; 
private void timer1_Tick (object sender, EventArgs e) 
{ 
     // update joystick information 
     m_CJoy.Update (); 
     // function to check whether the joystick is alive 
     FJoystick_Check_Alive ();

     // check joystick data 
     FJoystick_Check_Data ();

     // Represent joystick data as 3d data 
     m_C3d.OjwDraw (m_afAngle); //m_C3d.OjwDraw (); 
}
private void FJoystick_Check_Alive () 
{
    //Joystick Check 
            
    Color m_colorLive = Color.LightGreen; // 
    Color m_colorDead = Color.Gray; // Color of dead 
    if ( m_CJoy.IsValid == false) // 1)
    {
        //Indicates that the joystick is not connected 
if (lbJoystick.ForeColor != M_colorDead) 
        { 
            lbJoystick.Text = "Joystick (No Connected ) "; 
            lbJoystick.ForeColor = m_colorDead; 
        }

#region         Attempt to reconnect every 3 seconds 
        if ( m_CTmr_Joystick.Get ()> 3000 ) // Check if Joystick is dead (in 3 second units) // 2)
        { 
            Ojw.CMessage.Write ("Joystick Check again" ); 
            m_CJoy = new Ojw.CJoystick (Ojw.CJoystick._ID_0); // 3)

            if ( m_CJoy.IsValid == false) // 1)
            {
                Ojw.CMessage.Write("We can not find a joystick device in here. ");
                m_CTmr_Joystick.Set (); // Reinitialize the timer's counter. // 4)
            } 
            else Ojw.CMessage.Write ("Joystick is Connected"); 
        } 
    } 
    else 
    {
        //Indicates a connection 
if (lbJoystick.ForeColor != m_colorLive) 
        { 
            lbJoystick.Text = "Joystick (Connected)"; 
            lbJoystick.ForeColor = m_colorLive; 
        }
    } 
}

private void FJoystick_Check_Data()
{
    // top left of stick, 0+ left, 1+ - bottom 
    // pad 2+ left, 3+ - bottom 
    // stick right bottom, 4+ left, 5+ 
    // button 
    // Top 6 - 3: click 
    // Left 7 - 3: click 
    // right 8 - 3: click 
    // Bottom 9 - 3: click 
    // front right 10- 3: click 
    // Front left 11 - -3: Click
    // Front right lower 12-: Click 
    // Front left lower 13-: Click

    if (m_CJoy.IsValid == true)
    {
        float fDown = 10.0f;
        float fUp = 0.0f;
        int nNum = 0;
        // pad - check the difference between IsDown and IsDown_Event 
        nNum = 2;
        if (m_CJoy.IsDown(Ojw.CJoystick.PadKey.POVLeft) == true) m_afAngle[nNum] = -fDown;
        else if (m_CJoy.IsDown(Ojw.CJoystick.PadKey.POVRight) == true) m_afAngle[nNum] = fDown;
        else m_afAngle[nNum] = fUp;

        nNum = 3;
        if (m_CJoy.IsDown(Ojw.CJoystick.PadKey.POVUp) == true) m_afAngle[nNum] = -fDown;
        else if (m_CJoy.IsDown(Ojw.CJoystick.PadKey.POVDown) == true) m_afAngle[nNum] = fDown;
        else m_afAngle[nNum] = fUp;
        #endregion Pad

        #region stick // 2)
        // upper left joystick 
        m_afAngle[0] = (float)(20.0 * (m_CJoy.dX0 - 0.5));
        m_afAngle[1] = (float)(20.0 * (m_CJoy.dY0 - 0.5));

        // bottom right joystick 
        m_afAngle[4] = (float)(20.0 * (m_CJoy.dX1 - 0.5));
        m_afAngle[5] = (float)(20.0 * (m_CJoy.dY1 - 0.5));
        #endregion stick

        #region slide // 3)
        // slide 
        m_afAngle[13] = ((m_CJoy.Slide >= 0.5) ? (float)(20.0 * (0.5 - m_CJoy.Slide)) : 0.0f);
        m_afAngle[12] = ((m_CJoy.Slide <= 0.5) ? (float)(20.0 * (m_CJoy.Slide - 0.5)) : 0.0f);
        #endregion Slide

        #region         Button // 4)
        fDown = -3.0f;
        fUp = 0.0f;
        // Up 
        nNum = 6;
        if (m_CJoy.IsDown(Ojw.CJoystick.PadKey.Button4) == true) m_afAngle[nNum] = fDown;
        else m_afAngle[nNum] = fUp;
        // Left 
        nNum = 7;
        if (m_CJoy.IsDown(Ojw.CJoystick.PadKey.Button3) == true) m_afAngle[nNum] = fDown;
        else m_afAngle[nNum] = fUp;
        // Right 
        nNum = 8;
        if (m_CJoy.IsDown(Ojw.CJoystick.PadKey.Button2) == true) m_afAngle[nNum] = fDown;
        else m_afAngle[nNum] = fUp;
        // Down
        nNum = 9;
        if (m_CJoy.IsDown(Ojw.CJoystick.PadKey.Button1) == true) m_afAngle[nNum] = fDown;
        else m_afAngle[nNum] = fUp;

        // Front Left 
        nNum = 10;
        if (m_CJoy.IsDown(Ojw.CJoystick.PadKey.Button6) == true) m_afAngle[nNum] = fDown;
        else m_afAngle[nNum] = fUp;
        // Front Right 
        nNum = 11;
        if (m_CJoy.IsDown(Ojw.CJoystick.PadKey.Button5) == true) m_afAngle[nNum] = fDown;
        else m_afAngle[nNum] = fUp;
        #endregion Button

    }
}