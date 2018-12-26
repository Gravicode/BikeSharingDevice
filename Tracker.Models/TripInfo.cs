using System;
using Microsoft.SPOT;

namespace Tracker.Models
{
    public class TripInfo
    {
        public string UserName { get; set; }
        public string TripNumber { get; set; }
        public string DeviceName { get; set; }
        public bool IsActive { get; set; }
        public DateTime StartDate { get; set; }
        public DateTime EndDate { get; set; }
    }
}
