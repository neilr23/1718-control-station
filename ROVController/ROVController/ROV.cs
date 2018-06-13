using System;
using System.Timers;
using System.IO.Ports;
using Modbus.Device;

namespace ROVController
{
    public class ROV
    {
        private bool isConnected;
        private SerialPort serialPort;
        private IModbusSerialMaster modbus;
        private ushort[] registers;

        private Timer timer;
        private int loopCounter;
        private Thruster[] thrusters;
        private Avionics avionics;
        private Manipulator[] manipulators;
        private Relay[] relays;
        private VoltageSensor volt;

        public ROV(string portName, int baudRate, int updateInterval)
        {
            //set up thrusters, avionics, manipulator objects with
            //specific indices in register array
            thrusters = new Thruster[6];
            for (int i = 0; i < 6; i++)
            {
                thrusters[i] = new Thruster(14 + i, 20 + i, i);
            }

            avionics = new Avionics(9, 12);

            manipulators = new Manipulator[4];
            manipulators[0] = new Manipulator(6, false);
            manipulators[1] = new Manipulator(6, true);
            manipulators[2] = new Manipulator(7, false);
            manipulators[3] = new Manipulator(7, true);

            relays = new Relay[3];
            relays[0] = new Relay(13, 0);
            relays[1] = new Relay(13, 1);
            relays[2] = new Relay(13, 2);

            volt = new VoltageSensor(8);

            //connect to ROV computer
            serialPort = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One);
            serialPort.Open();
            modbus = ModbusSerialMaster.CreateRtu(serialPort);
            //(if no exceptions occur)
            isConnected = true;
            registers = new ushort[29];
            for (int i = 0; i < 29; i++)
            {
                registers[i] = 0;
            }

            //every x milliseconds send/receive data with ROV
            timer = new Timer(updateInterval);
            timer.AutoReset = true;
            timer.Elapsed += communicationTimerElapsed;
            timer.Start();
            loopCounter = 0;
        }

        //runs 100Hz
        private void communicationTimerElapsed(object o, ElapsedEventArgs ea)
        {
            loopCounter++;
            if (loopCounter >= 10)
            {
                lowPriorityCommunication();
                loopCounter = 0;
            }
            highPriorityCommunication();
        }

        //also runs 100Hz
        private void highPriorityCommunication()
        {
            //put speed numbers from thruster objects in modbus registers
            for (int i = 0; i < 6; i++)
            {
                thrusters[i].update(registers);
            }
            for (int i = 0; i < 4; i++)
            {
                manipulators[i].update(registers);
            }
            //send thruster and manipulator values
            writeRegisters(0, 8);
            //get voltage, avionics data into registers
            readRegisters(8, 5);
            //update objects with those register values
            avionics.update(registers);
            volt.update(registers);
        }

        //runs 10Hz
        private void lowPriorityCommunication()
        {
            //get relay states from objects into registers
            for (int i = 0; i < 2; i++)
            {
                relays[i].update(registers);
            }
            //send booleans for relays, lights
            writeRegisters(13, 1);
            //read thruster rpm, temp, ROV status and error codes
            readRegisters(14, 15);
            //put this data into objects from registers
            for (int i = 0; i < 6; i++)
            {
                thrusters[i].update(registers);
            }
        }

        //takes subset of surface register array and writes it to ROV register array
        private void writeRegisters(int start, int n)
        {
            ushort[] subset = new ushort[n];
            for (int i = 0; i < n; i++)
            {
                subset[i] = registers[start + i];
            }
            modbus.WriteMultipleRegisters(1, (ushort)start, subset);
        }

        //reads subset of ROV's register array and adds it to surface register array
        private void readRegisters(int start, int n)
        {
            ushort[] subset = modbus.ReadHoldingRegisters(1, (ushort)start, (ushort)n);
            for (int i = 0; i < n; i++)
            {
                registers[start + i] = subset[i];
            }
        }

        public bool Connected
        {
            get { return isConnected; }
            set
            {
                if (value)
                {
                    serialPort.Open();
                }
                else
                {
                    serialPort.Close();
                }
                isConnected = value;
            }
        }

        public byte getLastCommunicationError()
        {
            return (byte)(registers[28] >> 8);
        }

        public string getLastCommunicationErrorString()
        {
            /*From ModbusRtu.h, getLastError() returns the following values:
            ERR_NOT_MASTER = -1,
            ERR_POLLING = -2,
            ERR_BUFF_OVERFLOW = -3,
            ERR_BAD_CRC = -4,
            ERR_EXCEPTION = -5*/
            switch ((int)getLastCommunicationError())
            {
                case 0:
                    return "0: No Error";
                case -1:
                    return "-1: You are not a master device";
                case -2:
                    return "-2: Error polling slave";
                case -3:
                    return "-3: Slave received data buffer overflow";
                case -4:
                    return "-4: Bad checksum on message";
                case -5:
                    return "-5: Communication exception";
                default:
                    return getLastCommunicationError() + ": Unknown error";
            }
        }

        public byte getCommunicationErrorCount()
        {
            return (byte)registers[28];
        }

        public ushort getROVError()
        {
            return registers[27];
        }

        public string getROVErrorString()
        {
            switch(getROVError())
            {
                case 0:
                    return "0: No error";
                case 1:
                    return "1: Not connecting to IMU";
                case 2:
                    return "2: Not connecting to depth sensor";
                case 3:
                    return "3: Unable to connect to ESC";
                default:
                    return getROVError() + ": Unknown error";
            }
        }
    }
    public class Thruster
    {
        private int rpm = 0;
        private double temperature = 0;
        private int speed = 0;
        private int rpmAddr, tempAddr, speedAddr;
        public Thruster(int rpmAddr, int tempAddr, int speedAddr)
        {
            this.rpmAddr = rpmAddr;
            this.tempAddr = tempAddr;
            this.speedAddr = speedAddr;
        }
        public void update(ushort[] registers)
        {
            rpm = registers[rpmAddr];
            temperature = registers[tempAddr] / 10.0;
            registers[speedAddr] = (ushort)speed;
        }
        public int RPM { get { return rpm; } }
        public double Temperature { get { return temperature; } }
        public int Speed
        {
            get
            {
                return speed;
            }
            set
            {
                if (value < Int16.MinValue || value > Int16.MaxValue)
                {
                    throw new ArgumentOutOfRangeException();
                }
                speed = value;
            }
        }
        public bool Operational { get { return temperature != 0; } } //temperature set to 0 for ESC with lost connection
        public bool Active { get { return speed != 0; } }
        public bool Overheated { get { return temperature > 40.0; } }
    }
    public class Avionics
    {
        //yaw, pitch, and roll of ROV (degrees)
        private double[] ypr = { 0, 0, 0 };
        //depth (meters below surface)
        private double depth = 0.0;
        private int yprAddr, depthAddr;
        public Avionics(int yprAddr, int depthAddr)
        {
            this.yprAddr = yprAddr;
            this.depthAddr = depthAddr;
        }
        public void update(ushort[] registers)
        {
            for (int i = 0; i < 3; i++)
            {
                ypr[i] = (registers[yprAddr + i] * 360.0 / 65535.0) - 180.0; //map 0<>65535 to -180<>180
            }
            depth = registers[depthAddr];
        }
        public double Yaw { get { return ypr[0]; } }
        public double Pitch { get { return ypr[1]; } }
        public double Roll { get { return ypr[2]; } }
        public double Depth { get { return depth; } }
    }
    public class Manipulator
    {
        private int speed = 0;
        private int speedAddr;
        private bool highByte;
        public Manipulator(int speedAddr, bool highByte)
        {
            this.speedAddr = speedAddr;
            this.highByte = highByte;
        }
        public void update(ushort[] registers)
        {
            if (highByte)
            {
                registers[speedAddr] = (ushort)((ushort)(speed) << 8);
            }
            else
            {
                registers[speedAddr] = (ushort)speed;
            }
        }
        public int Speed
        {
            get
            {
                return speed;
            }
            set
            {
                if (speed < SByte.MinValue || speed > SByte.MaxValue)
                {
                    throw new ArgumentOutOfRangeException();
                }
                speed = value;
            }
        }
        public bool Active { get { return speed != 0; } }
    }
    public class Relay
    {
        private bool state;
        private int stateAddr;
        private int stateBitAddr;
        public Relay(int stateAddr, int stateBitAddr)
        {
            this.stateAddr = stateAddr;
            this.stateBitAddr = stateBitAddr;
        }
        public void update(ushort[] registers)
        {
            if (state)
            {
                registers[stateAddr] |= (ushort)(1 << stateBitAddr);
            }
            else
            {
                registers[stateAddr] &= (ushort)~(1 << stateBitAddr);
            }
        }
        public bool State { get { return state; } set { state = value; } }
    }
    public class VoltageSensor
    {
        private double voltage;
        private int voltAddr;
        public VoltageSensor(int voltAddr)
        {
            this.voltAddr = voltAddr;
        }
        public void update(ushort[] registers)
        {
            voltage = registers[voltAddr] * 5.0 / 1023 * 5.7; //5v per 1023 ADC ticks and a 10k/47k voltage divider
        }
        public double Voltage { get { return voltage; } }
    }
}
