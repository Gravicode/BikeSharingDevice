using System;
using Microsoft.SPOT;

namespace Tracker.Models
{
    public class SensorData
    {
        public GpsPoint Position { get; set; }
        public bool SOS { get; set; }
        public bool IsLocked { get; set; }
        public DateTime TimeStamp { get; set; }
    }
}
