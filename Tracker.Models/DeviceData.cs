using System;
using Microsoft.SPOT;

namespace Tracker.Models
{
    public class DeviceData
    {
        public DeviceData()
        {
            this.Position = new GpsPoint();
        }
        public bool IsLocked { get; set; }
        public DateTime TimeStamp { get; set; }
        public GpsPoint Position { get; set; }
        public bool SOS { get; set; }
        public TripInfo Info { get; set; }
    }
}
