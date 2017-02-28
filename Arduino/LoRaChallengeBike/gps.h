#ifndef _gps_h_
#define _gps_h_

typedef struct {
  float latitude;
  float longitude;
  uint timeHH;
  uint timeMM;
  uint timeSS;
  uint satellites;
} gps_data_t;

#endif /*_gps_h_*/
