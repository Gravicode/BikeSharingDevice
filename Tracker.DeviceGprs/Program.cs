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
using uPLibrary.Networking.M2Mqtt;
using Microsoft.SPOT.Net.NetworkInformation;
using GHI.Networking;
using Tracker.Models;
using System.Net;
using System.Text;
using uPLibrary.Networking.M2Mqtt.Messages;
using System.IO.Ports;
using Json.NETMF;

namespace Tracker.DeviceGprs
{
    public partial class Program
    {
        static int TimerCountdown;
        static string DeviceName = "KayuhBike1";
        static TripInfo Info;
        static bool LastSOS = false;
        static GpsPoint LastPosition = null;
       
        const string MQTT_BROKER_ADDRESS = "13.76.142.227";
        public static MqttClient client { set; get; }
        string clientId = Guid.NewGuid().ToString();
        string username = "mifmasterz";
        string password = "123qweasd";
        string DataTopic = "mifmasterz/BMC/data";
        string ControlTopic = "mifmasterz/BMC/control";
        // This method is run when the mainboard is powered up or reset.   
        void ProgramStarted()
        {
            characterDisplay.Clear();
            characterDisplay.Print("DEVICE INIT");
            characterDisplay.BacklightEnabled = true;
            NetworkChange.NetworkAvailabilityChanged += (a, b) => Debug.Print("Network availability changed: " + b.IsAvailable.ToString());
            NetworkChange.NetworkAddressChanged += (a, b) => Debug.Print("Network address changed");

            this.cellularRadio.PowerOn();
            this.cellularRadio.LineReceived += cellularRadio_LineReceived;
            //for sending data through gprs
            var th1 = new Thread(new ThreadStart(TelemetryLoop));
            th1.Start();
           
            button.ButtonReleased += button_ButtonReleased;
            StartGPS();
        }

        void CountDown()
        {
            while (true)
            {
                if (TimerCountdown > 0)
                {
                    TimerCountdown--;
                    var ts = new TimeSpan(0, 0, TimerCountdown);
                    Debug.Print("Countdown timer : "+ts.Hours+":"+ts.Minutes+":"+ts.Seconds);
                }
                else
                {
                    Debug.Print("Time is up, please return this bike to station..");
                }
                Thread.Sleep(1000);
                if (Info != null && !Info.IsActive) break;
            }
        }

        void button_ButtonReleased(GTM.GHIElectronics.Button sender, GTM.GHIElectronics.Button.ButtonState state)
        {
            characterDisplay.Clear();
            characterDisplay.Print("SOS Pressed");
            LastSOS = true;
        }

        void TelemetryLoop()
        {
            characterDisplay.Clear();
            characterDisplay.Print("Waiting for modem in 15 secs");
            Thread.Sleep(15000);

            this.cellularRadio.UseThisNetworkInterface("3gprs", "3gprs", "3gprs", PPPSerialModem.AuthenticationType.Pap);
            characterDisplay.Clear();
            characterDisplay.Print("Waiting for DHCP");
            while (this.cellularRadio.NetworkInterface.IPAddress == "0.0.0.0")
            {
                Debug.Print("Waiting on DHCP");
                Thread.Sleep(250);
            }
            characterDisplay.Clear();
            characterDisplay.Print("IP:" + this.cellularRadio.NetworkInterface.IPAddress);
            client = new MqttClient(IPAddress.Parse(MQTT_BROKER_ADDRESS));

            client.Connect(clientId, username, password);

            SubscribeMessage();
            Random rnd = new Random();
            while (true)
            { 
                if (Info != null && Info.IsActive )
                {
                    var item = new DeviceData() { Info = Info, TimeStamp = DateTime.Now, IsLocked = false, Position = LastPosition == null ? new GpsPoint() { Latitude=0, Longitude=0 } : LastPosition, SOS = LastSOS };
                    LastSOS = false;
                    PublishMessage(DataTopic, Json.NETMF.JsonSerializer.SerializeObject(item));
                }
                Thread.Sleep(2000);

            }
            client.Disconnect();
            //loop forever



        }

        private void DoAction(string data)
        {
            Hashtable hashTable = JsonSerializer.DeserializeString(data) as Hashtable;

            //var objX = Json.NETMF.JsonSerializer.DeserializeString(data) as DictionaryEntry;
            var action = new DeviceAction() { ActionName = hashTable["ActionName"].ToString(), Params0 = hashTable["Params0"] == null ? "" : hashTable["Params0"].ToString(), Params1 = hashTable["Params1"] == null ? "" : hashTable["Params1"].ToString() };
            // Check the payload is a single integer value
            if (action != null)
            {
              
                switch (action.ActionName)
                {
                    case "Start":
                        //unlock
                        if (Info != null && Info.IsActive)
                        {
                            Debug.Print("Bike is already started / unlocked");
                            break;
                        }
                        else
                        {
                            Debug.Print("Bike is unlocked/started");
                            relayX1.TurnOn();
                        }
                        TimerCountdown = 30 * 60;
                        Info = new TripInfo();
                        Info.TripNumber = action.Params0;
                        Info.UserName = action.Params1;
                        Info.StartDate = DateTime.Now;
                        Info.EndDate = DateTime.Now;
                        Info.DeviceName = DeviceName;
                        Info.IsActive = true;
                        var CountDownThread = new Thread(new ThreadStart(CountDown));
                        CountDownThread.Start();

                        break;
                    case "Stop":
                        //lock
                        if (Info != null && Info.IsActive)
                        {
                            Info.EndDate = DateTime.Now;
                            Info.IsActive = false;
                            //var item = new DeviceData() { TimeStamp = DateTime.Now, Info = Info, SOS = LastSOS, Position = LastPosition };
                            relayX1.TurnOff();
                            Debug.Print("Bike is stopped/locked");
                        }
                        else
                        {
                            Debug.Print("Bike is already stopped/locked");

                        }
                        break;
                    case "SOS":
                        LastSOS = true;
                        Debug.Print("sos is turned on");
                        break;

                }
                
                
            }
            
        }

        void cellularRadio_LineReceived(GTM.GHIElectronics.CellularRadio sender, string line)
        {
            Debug.Print(line);
        }

        void PublishMessage(string Topic, string Pesan)
        {
            client.Publish(Topic, Encoding.UTF8.GetBytes(Pesan), MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE, false);
            characterDisplay.Clear();
            characterDisplay.Print("SENDING: " + DateTime.Now.ToString("HH:mm:ss"));
        }

        void SubscribeMessage()
        {
            // register to message received 
            client.MqttMsgPublishReceived += client_MqttMsgPublishReceived;
            client.Subscribe(new string[] { DataTopic, ControlTopic }, new byte[] { MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE, MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE });

        }

         void client_MqttMsgPublishReceived(object sender, MqttMsgPublishEventArgs e)
        {
            
            // handle message received 
            string Message = new string(Encoding.UTF8.GetChars(e.Message));

            Debug.Print("Message Received : " + Message);
            if (e.Topic == ControlTopic)
            {
                DoAction(Message);
            }

        }

        public void StartGPS()
        {
            //Lcd.BacklightBrightness = 100;

            SerialPort serialPort = new SerialPort("COM1", 9600, Parity.None, 8, StopBits.One);

            //OutputPort powerPin = new OutputPort(Pins.GPIO_PIN_D2, false);

            Reader gpsShield = new Reader(serialPort, 100, 0.0);
            gpsShield.GpsData += GpsShield_GpsData;
            gpsShield.Start();
            characterDisplay.Clear();
            characterDisplay.Print("GPS STARTED");
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
            LastPosition = gpsPoint;
            characterDisplay.Clear();
            characterDisplay.SetCursorPosition(0, 0);
            characterDisplay.Print("lat:" + lat);
            characterDisplay.SetCursorPosition(1, 0);
            characterDisplay.Print("lng:" + lon);
            WriteLine("Lat: " + lat + "     ");
            WriteLine("Lng: " + lon + "     ");
        }

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
}
