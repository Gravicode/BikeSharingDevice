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
using Microsoft.SPOT.Net.NetworkInformation;

using GHI.Networking;
using System.Net;
using System.Text;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;
using Microsoft.SPOT.Hardware;
using Tracker.Models;

namespace CellularRadio
{
    public partial class Program
    {
        const string MQTT_BROKER_ADDRESS = "13.76.142.227";
     
        public static MqttClient client { set; get; }
        string clientId = Guid.NewGuid().ToString();
        string username = "mifmasterz";
        string password = "123qweasd";
        string DataTopic = "mifmasterz/BMC/data";
        // This method is run when the mainboard is powered up or reset.   
        void ProgramStarted()
        {
            NetworkChange.NetworkAvailabilityChanged += (a, b) => Debug.Print("Network availability changed: " + b.IsAvailable.ToString());
            NetworkChange.NetworkAddressChanged += (a, b) => Debug.Print("Network address changed");

            this.cellularRadio.PowerOn();
            this.cellularRadio.LineReceived += cellularRadio_LineReceived;

            new Thread(() =>
            {

                Thread.Sleep(15000);
                //this.cellularRadio.SendSms("+628174810345", "test");
                //this.cellularRadio.AttachGprs("internet", "wap", "wap123");//.
                this.cellularRadio.UseThisNetworkInterface("3gprs", "3gprs", "3gprs", PPPSerialModem.AuthenticationType.Pap);

                while (this.cellularRadio.NetworkInterface.IPAddress == "0.0.0.0")
                {
                    Debug.Print("Waiting on DHCP");
                    Thread.Sleep(250);
                }
       
                client = new MqttClient(IPAddress.Parse(MQTT_BROKER_ADDRESS));
              
                client.Connect(clientId, username,password);
             
                SubscribeMessage();
                Random rnd = new Random();
                while (true)
                {
                    var item = new SensorData() { IsLocked = false, Position = new GpsPoint() { Latitude = rnd.NextDouble(), Longitude = rnd.NextDouble() }, SOS = false };
                    
                    PublishMessage(DataTopic, Json.NETMF.JsonSerializer.SerializeObject(item));
                    Thread.Sleep(2000);
                }
                client.Disconnect();
                //loop forever
                Thread.Sleep(Timeout.Infinite);
                /*
                using (var req = HttpWebRequest.Create("http://www.ghielectronics.com/") as HttpWebRequest)
                {
                    req.KeepAlive = false;
                    req.ContentLength = 0;

                    using (var res = req.GetResponse() as HttpWebResponse)
                    {
                        using (var stream = res.GetResponseStream())
                        {
                            int read = 0, total = 0;
                            var buffer = new byte[1024];

                            do
                            {
                                read = stream.Read(buffer, 0, buffer.Length);
                                total += read;

                                Thread.Sleep(20);
                            }
                            while (read != 0);
                            Debug.Print(new string(Encoding.UTF8.GetChars(buffer)));
                        }
                    }
                }*/
            }).Start();

          
        }

        void cellularRadio_LineReceived(GTM.GHIElectronics.CellularRadio sender, string line)
        {
            Debug.Print(line);
        }

        void PublishMessage(string Topic, string Pesan)
        {
            client.Publish(Topic, Encoding.UTF8.GetBytes(Pesan), MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE, false);
        }

        void SubscribeMessage()
        {
            // register to message received 
            client.MqttMsgPublishReceived += client_MqttMsgPublishReceived;
            client.Subscribe(new string[] { DataTopic }, new byte[] { MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE });

        }

        static void client_MqttMsgPublishReceived(object sender, MqttMsgPublishEventArgs e)
        {
            // handle message received 
            string Message = new string(Encoding.UTF8.GetChars(e.Message));
           
            Debug.Print("Message Received : " + Message);
            
        }
    }
}
