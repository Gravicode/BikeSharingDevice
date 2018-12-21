using System;
using System.Collections;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Presentation;
using Microsoft.SPOT.Presentation.Controls;
using Microsoft.SPOT.Presentation.Media;
using Microsoft.SPOT.Presentation.Shapes;
using Microsoft.SPOT.Touch;

using Gadgeteer.Networking;
using GT = Gadgeteer;
using GTM = Gadgeteer.Modules;
using Gadgeteer.Modules.GHIElectronics;
using System.IO.Ports;
using Microsoft.SPOT.Hardware;
using System.Text;

namespace GPS
{
    public partial class Program
    {
        int Counter = 0;
        static SensorData LatestState = new SensorData() { IsLocked=false, Latitude=0, Longitude=0, SOS=false };
        
        SimpleSerial UART = null;
        // This method is run when the mainboard is powered up or reset.   
        void ProgramStarted()
        {
            /*******************************************************************************************
            Modules added in the Program.gadgeteer designer view are used by typing 
            their name followed by a period, e.g.  button.  or  camera.
            
            Many modules generate useful events. Type +=<tab><tab> to add a handler to an event, e.g.:
                button.ButtonPressed +=<tab><tab>
            
            If you want to do something periodically, use a GT.Timer and handle its Tick event, e.g.:
                GT.Timer timer = new GT.Timer(1000); // every second (1000ms)
                timer.Tick +=<tab><tab>
                timer.Start();
            *******************************************************************************************/


            // Use Debug.Print to show messages in Visual Studio's "Output" window during debugging.
            Debug.Print("Program Started");
            characterDisplay.Print("MULAI...");
            characterDisplay.BacklightEnabled = true;
            button.ButtonReleased += button_ButtonReleased;
            StartGPS();
            StartLORA();
            
        }

        void button_ButtonReleased(GTM.GHIElectronics.Button sender, GTM.GHIElectronics.Button.ButtonState state)
        {
            LatestState.SOS = true;
        }
        void StartLORA()
        {
            UART = new SimpleSerial("COM4", 57600); //socket 9
            UART.ReadTimeout = 0;
            UART.DataReceived += UART_DataReceived;
            Debug.Print("57600");
            Debug.Print("RN2483 Test");
            PrintToLcd("RN2483 Test");
            OutputPort reset = new OutputPort(GHI.Pins.FEZSpiderII.Socket9.Pin6, false);
            OutputPort reset2 = new OutputPort(GHI.Pins.FEZSpiderII.Socket9.Pin3, false);

            reset.Write(true);
            reset2.Write(true);

            Thread.Sleep(100);
            reset.Write(false);
            reset2.Write(false);

            Thread.Sleep(100);
            reset.Write(true);
            reset2.Write(true);

            Thread.Sleep(100);

            waitForResponse();

            sendCmd("sys factoryRESET");
            sendCmd("sys get hweui");
            sendCmd("mac get deveui");
            Thread.Sleep(3000);
            // For TTN
            sendCmd("mac set devaddr AAABBBEE");  // Set own address
            Thread.Sleep(3000);
            sendCmd("mac set appskey 2B7E151628AED2A6ABF7158809CF4F3D");
            Thread.Sleep(3000);

            sendCmd("mac set nwkskey 2B7E151628AED2A6ABF7158809CF4F3D");
            Thread.Sleep(3000);

            sendCmd("mac set adr off");
            Thread.Sleep(3000);

            sendCmd("mac set rx2 3 868400000");//869525000
            Thread.Sleep(3000);

            sendCmd("mac join abp");
            Thread.Sleep(3000);
            sendCmd("mac get status");
            sendCmd("mac get devaddr");
            Thread.Sleep(2000);

            Thread th1 = new Thread(new ThreadStart(Loop));
            th1.Start();

        }
        
        public  void StartGPS()
        {
            //Lcd.BacklightBrightness = 100;

            SerialPort serialPort = new SerialPort("COM1", 9600, Parity.None, 8, StopBits.One);

            //OutputPort powerPin = new OutputPort(Pins.GPIO_PIN_D2, false);

            Reader gpsShield = new Reader(serialPort, 100, 0.0);
            gpsShield.GpsData += GpsShield_GpsData;
            gpsShield.Start();

            /*
            while (true)
            {
                // Do other processing here.
                //
                // Can stop the gps processing by calling...
                //  gpsShield.Stop();
                //
                // Restart by calling...
                //  gpsShield.Start();
                //
                Debug.Print("Main...");

                Thread.Sleep(10000);
            }*/
        }
        void WriteLine(string Message, bool Status = false)
        {
            Debug.Print(Message);
        }
        private void GpsShield_GpsData(GpsPoint gpsPoint)
        {
            Debug.Print("time: " + gpsPoint.Timestamp + "Lat/Lng: " + gpsPoint.Latitude + "/" + gpsPoint.Longitude);
            //Lcd.Clear();
            WriteLine("  GPS MONITOR ", false);
            WriteLine("              ", false);
            WriteLine("Time: " + gpsPoint.Timestamp + " UTC     ");
            string lat = gpsPoint.Latitude.ToString("F4");
            string lon = gpsPoint.Longitude.ToString("F4");
            LatestState.Latitude = gpsPoint.Latitude;
            LatestState.Longitude = gpsPoint.Longitude;
            characterDisplay.Clear();
            characterDisplay.SetCursorPosition(0, 0);
            characterDisplay.Print("lat:" + lat);
            characterDisplay.SetCursorPosition(1, 0);
            characterDisplay.Print("lng:" + lon);
            WriteLine("Lat: " + lat + "     ");
            WriteLine("Lng: " + lon + "     ");
        }


        private static string[] _dataInLora;
        private static string rx;


        void UART_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {

            _dataInLora = UART.Deserialize();
            for (int index = 0; index < _dataInLora.Length; index++)
            {
                rx = _dataInLora[index];
                //if error
                if (_dataInLora[index].Length > 5)
                {

                    //if receive data
                    if (rx.Substring(0, 6) == "mac_rx")
                    {
                        string hex = _dataInLora[index].Substring(9);

                        //update display
                        //txtMessage.Text = hex;//Unpack(hex);
                        //txtMessage.Invalidate();
                        //window.Invalidate();
                        byte[] data = StringToByteArrayFastest(hex);
                        string decoded = new String(UTF8Encoding.UTF8.GetChars(data));
                        Debug.Print("decoded:" + decoded);

                    }
                }
            }
            Debug.Print(rx);
        }

        public static byte[] StringToByteArrayFastest(string hex)
        {
            if (hex.Length % 2 == 1)
                throw new Exception("The binary key cannot have an odd number of digits");

            byte[] arr = new byte[hex.Length >> 1];

            for (int i = 0; i < hex.Length >> 1; ++i)
            {
                arr[i] = (byte)((GetHexVal(hex[i << 1]) << 4) + (GetHexVal(hex[(i << 1) + 1])));
            }

            return arr;
        }

        public static int GetHexVal(char hex)
        {
            int val = (int)hex;
            //For uppercase A-F letters:
            return val - (val < 58 ? 48 : 55);
            //For lowercase a-f letters:
            //return val - (val < 58 ? 48 : 87);
            //Or the two combined, but a bit slower:
            //return val - (val < 58 ? 48 : (val < 97 ? 55 : 87));
        }

        private static void OnTap(object sender)
        {
            Debug.Print("Button tapped.");
        }

        void Loop()
        {
            int counter = 0;
            while (true)
            {
                counter++;
               
                var jsonStr = Json.NETMF.JsonSerializer.SerializeObject(LatestState);
                Debug.Print("kirim :" + jsonStr);
                PrintToLcd("send count: " + counter);
                sendData(jsonStr);
               
                Thread.Sleep(5000);
                byte[] rx_data = new byte[20];

                if (UART.CanRead)
                {
                    var count = UART.Read(rx_data, 0, rx_data.Length);
                    if (count > 0)
                    {
                        Debug.Print("count:" + count);
                        var hasil = new string(System.Text.Encoding.UTF8.GetChars(rx_data));
                        Debug.Print("read:" + hasil);

                        //mac_rx 2 AABBCC
                    }
                }
                var TimeStr = DateTime.Now.ToString("dd/MM/yy HH:mm");
                //insert to db
            
                Counter++;


                Thread.Sleep(2000);
                LatestState.SOS = false;
            }

        }


        void PrintToLcd(string Message)
        {
            characterDisplay.Clear();
            characterDisplay.Print(Message);
            Debug.Print(Message);
        }

        void sendCmd(string cmd)
        {
            byte[] rx_data = new byte[20];
            Debug.Print(cmd);
            Debug.Print("\n");
            // flush all data
            UART.Flush();
            // send some data
            var tx_data = Encoding.UTF8.GetBytes(cmd);
            UART.Write(tx_data, 0, tx_data.Length);
            tx_data = Encoding.UTF8.GetBytes("\r\n");
            UART.Write(tx_data, 0, tx_data.Length);
            Thread.Sleep(100);
            while (!UART.IsOpen)
            {
                UART.Open();
                Thread.Sleep(100);
            }
            if (UART.CanRead)
            {
                var count = UART.Read(rx_data, 0, rx_data.Length);
                if (count > 0)
                {
                    Debug.Print("count cmd:" + count);
                    var hasil = new string(System.Text.Encoding.UTF8.GetChars(rx_data));
                    Debug.Print("read cmd:" + hasil);
                }
            }
        }

        void waitForResponse()
        {
            byte[] rx_data = new byte[20];

            while (!UART.IsOpen)
            {
                UART.Open();
                Thread.Sleep(100);
            }
            if (UART.CanRead)
            {
                var count = UART.Read(rx_data, 0, rx_data.Length);
                if (count > 0)
                {
                    Debug.Print("count res:" + count);
                    var hasil = new string(System.Text.Encoding.UTF8.GetChars(rx_data));
                    Debug.Print("read res:" + hasil);
                }

            }
        }

        public static string Unpack(string input)
        {
            byte[] b = new byte[input.Length / 2];

            for (int i = 0; i < input.Length; i += 2)
            {
                b[i / 2] = (byte)((FromHex(input[i]) << 4) | FromHex(input[i + 1]));
            }
            return new string(Encoding.UTF8.GetChars(b));
        }

        public static int FromHex(char digit)
        {
            if ('0' <= digit && digit <= '9')
            {
                return (int)(digit - '0');
            }

            if ('a' <= digit && digit <= 'f')
                return (int)(digit - 'a' + 10);

            if ('A' <= digit && digit <= 'F')
                return (int)(digit - 'A' + 10);

            throw new ArgumentException("digit");
        }

        char getHexHi(char ch)
        {
            int nibbleInt = ch >> 4;
            char nibble = (char)nibbleInt;
            int res = (nibble > 9) ? nibble + 'A' - 10 : nibble + '0';
            return (char)res;
        }

        char getHexLo(char ch)
        {
            int nibbleInt = ch & 0x0f;
            char nibble = (char)nibbleInt;
            int res = (nibble > 9) ? nibble + 'A' - 10 : nibble + '0';
            return (char)res;
        }

        void sendData(string msg)
        {
            byte[] rx_data = new byte[20];
            char[] data = msg.ToCharArray();
            Debug.Print("mac tx uncnf 1 ");
            var tx_data = Encoding.UTF8.GetBytes("mac tx uncnf 1 ");
            UART.Write(tx_data, 0, tx_data.Length);

            // Write data as hex characters
            foreach (char ptr in data)
            {
                tx_data = Encoding.UTF8.GetBytes(new string(new char[] { getHexHi(ptr) }));
                UART.Write(tx_data, 0, tx_data.Length);
                tx_data = Encoding.UTF8.GetBytes(new string(new char[] { getHexLo(ptr) }));
                UART.Write(tx_data, 0, tx_data.Length);


                Debug.Print(new string(new char[] { getHexHi(ptr) }));
                Debug.Print(new string(new char[] { getHexLo(ptr) }));
            }
            tx_data = Encoding.UTF8.GetBytes("\r\n");
            UART.Write(tx_data, 0, tx_data.Length);
            Debug.Print("\n");
            Thread.Sleep(5000);

            if (UART.CanRead)
            {
                var count = UART.Read(rx_data, 0, rx_data.Length);
                if (count > 0)
                {
                    Debug.Print("count after:" + count);
                    var hasil = new string(System.Text.Encoding.UTF8.GetChars(rx_data));
                    Debug.Print("read after:" + hasil);
                }
            }
        }
    }

    public class GpsPoint
    {
        public DateTime Timestamp { get; set; }
        public double Latitude { get; set; }
        public double Longitude { get; set; }
        public double SpeedInKnots { get; set; }
        public double BearingInDegrees { get; set; }
    }

    public class Reader
    {
        private readonly object _lock = new object();
        private readonly SerialPort _serialPort;
        private readonly int _timeOut;
        private readonly double _minDistanceBetweenPoints;
        private bool _isStarted;
        private Thread _processor;

        public delegate void LineProcessor(string line);

        public delegate void GpsDataProcessor(GpsPoint gpsPoint);

        public event LineProcessor RawLine;
        public event GpsDataProcessor GpsData;

        public bool IsStarted { get { return _isStarted; } }

        public Reader(SerialPort serialPort)
            : this(serialPort, 100, 0.0)
        {

        }
        public Reader(SerialPort serialPort, int timeOutBetweenReadsInMilliseconds, double minDistanceInMilesBetweenPoints)
        {
            _serialPort = serialPort;
            _timeOut = timeOutBetweenReadsInMilliseconds;
            _minDistanceBetweenPoints = minDistanceInMilesBetweenPoints;
        }

        public bool Start()
        {
            lock (_lock)
            {
                if (_isStarted)
                {
                    return false;
                }
                _isStarted = true;
                _processor = new Thread(ThreadProc);
                _processor.Start();
            }
            return true;
        }

        public bool Stop()
        {
            lock (_lock)
            {
                if (!_isStarted)
                {
                    return false;
                }
                _isStarted = false;
                if (!_processor.Join(5000))
                {
                    _processor.Abort();
                }
                return true;
            }
        }

        private void ThreadProc()
        {
            Debug.Print("GPS thread started...");
            if (!_serialPort.IsOpen)
            {
                _serialPort.Open();
            }
            while (_isStarted)
            {
                int bytesToRead = _serialPort.BytesToRead;
                if (bytesToRead > 0)
                {
                    byte[] buffer = new byte[bytesToRead];
                    _serialPort.Read(buffer, 0, buffer.Length);
                    try
                    {
                        string temp = new string(System.Text.Encoding.UTF8.GetChars(buffer));
                        Debug.Print(temp);
                        ProcessBytes(temp);
                    }
                    catch (Exception ex)
                    {
                        // only process lines we can parse.
                        Debug.Print(ex.ToString());
                    }
                }

                Thread.Sleep(_timeOut);
            }
            Debug.Print("GPS thread stopped...");
        }

        private string _data = string.Empty;
        private GpsPoint _lastPoint;
        private DateTime _lastDateTime = DateTime.Now;

        private void ProcessBytes(string temp)
        {
            while (temp.IndexOf('\n') != -1)
            {
                string[] parts = temp.Split('\n');
                _data += parts[0];
                _data = _data.Trim();
                if (_data != string.Empty)
                {
                    if (_data.IndexOf("$GPRMC") == 0)
                    {
                        Debug.Print("GOT $GPRMC LINE");
                        if (GpsData != null)
                        {
                            GpsPoint gpsPoint = GprmcParser.Parse(_data);
                            if (gpsPoint != null)
                            {
                                bool isOk = true;
                                if (_lastPoint != null)
                                {
                                    double distance = GeoDistanceCalculator.DistanceInMiles(gpsPoint.Latitude, gpsPoint.Longitude,
                                                                                   _lastPoint.Latitude, _lastPoint.Longitude);
                                    double distInFeet = distance * 5280;
                                    Debug.Print("distance = " + distance + " mi (" + distInFeet + " feet)");
                                    if (distance < _minDistanceBetweenPoints)
                                    {
                                        // Too close to the last point....don't raise the event
                                        isOk = false;
                                    }
                                    DateTime now = DateTime.Now;
                                    TimeSpan diffseconds = (now - _lastDateTime);
                                    if (diffseconds.Seconds > 60)
                                    {
                                        // A minute has gone by, so update
                                        isOk = true;
                                        _lastDateTime = now;
                                    }
                                }
                                _lastPoint = gpsPoint;

                                // Raise the event
                                if (isOk)
                                {
                                    GpsData(gpsPoint);
                                }
                            }
                        }
                    }
                    if (RawLine != null)
                    {
                        RawLine(_data);
                    }
                }
                temp = parts[1];
                _data = string.Empty;
            }
            _data += temp;
        }
     
    }

    public class SensorData
    {
        public double Longitude { get; set; }
        public double Latitude { get; set; }
        public bool SOS { get; set; }
        public bool IsLocked { get; set; }
    }

    public class GprmcParser
    {
        // Parse the GPRMC line
        //
        public static GpsPoint Parse(string line)
        {
            // $GPRMC,040302.663,A,3939.7,N,10506.6,W,0.27,358.86,200804,,*1A
            //Debug.Print("GpsPoint Parse");                                                                                                                                                                                                                           
            if (!IsCheckSumGood(line))
            {
                return null;
            }
            //Debug.Print(line);
            try                                                                                                                         
            {
                string[] parts = line.Split(',');
                //Debug.Print(parts.Length.ToString());
                if (parts.Length != 13)  // This GPS has extra field
                {
                    return null;
                }
                //Debug.Print(parts[2]);
                if (parts[2] != "A")
                {
                    return null;
                }
                //Debug.Print(parts[9]);
                string date = parts[9]; // UTC Date DDMMYY
                if (date.Length != 6)
                {
                    return null;
                }
                int year = 2000 + int.Parse(date.Substring(4, 2));
                int month = int.Parse(date.Substring(2, 2));
                int day = int.Parse(date.Substring(0, 2));
                string time = parts[1]; // HHMMSS.XXX
                if (time.Length != 9)
                {
                    return null;
                }
                int hour = int.Parse(time.Substring(0, 2));
                int minute = int.Parse(time.Substring(2, 2));
                int second = int.Parse(time.Substring(4, 2));
                int milliseconds = int.Parse(time.Substring(7, 2));
                DateTime utcTime = new DateTime(year, month, day, hour, minute, second, milliseconds);

                string lat = parts[3];  // HHMM.MMMM
                if (lat.Length != 10)
                {
                    return null;
                }
                double latHours = double.Parse(lat.Substring(0, 2));
                double latMins = double.Parse(lat.Substring(2));
                double latitude = latHours + latMins / 60.0;
                if (parts[4] == "S")       // N or S
                {
                    latitude = -latitude;
                }

                string lng = parts[5];  // HHHMM.MMMMM
                if (lng.Length != 11)
                {
                    return null;
                }
                double lngHours = double.Parse(lng.Substring(0, 3));
                double lngMins = double.Parse(lng.Substring(3));
                double longitude = lngHours + lngMins / 60.0;
                if (parts[6] == "W")
                {
                    longitude = -longitude;
                }

                double speed = double.Parse(parts[7]);
                double bearing = double.Parse(parts[8]);

                // Should probably validate check sum

                GpsPoint gpsPoint = new GpsPoint
                {
                    BearingInDegrees = bearing,
                    Latitude = latitude,
                    Longitude = longitude,
                    SpeedInKnots = speed,
                    Timestamp = utcTime
                };
                return gpsPoint;

            }
            catch (Exception)
            {
                // One of our parses failed...ignore.
                Debug.Print("parse exception");
            }
            return null;
        }

        private static bool IsCheckSumGood(string sentence)
        {
            int index1 = sentence.IndexOf("$");
            int index2 = sentence.LastIndexOf("*");

            if (index1 != 0 || index2 != sentence.Length - 3)
            {
                return false;
            }

            string checkSumString = sentence.Substring(index2 + 1, 2);
            int checkSum1 = Convert.ToInt32(checkSumString, 16);

            string valToCheck = sentence.Substring(index1 + 1, index2 - 1);
            char c = valToCheck[0];
            for (int i = 1; i < valToCheck.Length; i++)
            {
                c ^= valToCheck[i];
            }

            return checkSum1 == c;
        }
    }

    public static class GeoDistanceCalculator
    {
        private const double _earthRadiusInMiles = 3956.0;
        private const double _earthRadiusInKilometers = 6367.0;
        public static double DistanceInMiles(double lat1, double lng1, double lat2, double lng2)
        {
            return Distance(lat1, lng1, lat2, lng2, _earthRadiusInMiles);
        }
        public static double DistanceInKilometers(double lat1, double lng1, double lat2, double lng2)
        {
            return Distance(lat1, lng1, lat2, lng2, _earthRadiusInKilometers);
        }
        private static double Distance(double lat1, double lng1, double lat2, double lng2, double radius)
        {
            // Implements the Haversine formulat http://en.wikipedia.org/wiki/Haversine_formula
            //
            var lat = NumericExtensions.ToRadians(lat2 - lat1);
            var lng = NumericExtensions.ToRadians(lng2 - lng1);
            var sinLat = System.Math.Sin(0.5 * lat);
            var sinLng = System.Math.Sin(0.5 * lng);
            var cosLat1 = System.Math.Cos(NumericExtensions.ToRadians(lat1));
            var cosLat2 = System.Math.Cos(NumericExtensions.ToRadians(lat2));
            var h1 = sinLat * sinLat + cosLat1 * cosLat2 * sinLng * sinLng;
            var h2 = System.Math.Sqrt(h1);
            var h3 = 2 * System.Math.Asin(System.Math.Min(1, h2));
            return radius * h3;
        }
    }
    public static class NumericExtensions
    {
        public static double ToRadians(this double val)
        {
            return (System.Math.PI / 180) * val;
        }
    }
}
