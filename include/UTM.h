/* -*- mode: C++ -*-
 *
 *  Conversions between Latitude/Longitude and UTM
 *              (Universal Transverse Mercator) coordinates.
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _UTM_H
#define _UTM_H

/**  @file
   
     @brief Universal Transverse Mercator transforms.

     Functions to convert (spherical) latitude and longitude to and
     from (Euclidean) UTM coordinates.

     @author Chuck Gantz- chuck.gantz@globalstar.com
 */

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>


/** Unit conversion constants: */
const double INCHES_PER_FOOT = 12.0;
const double CM_PER_INCH = 2.54;
const double CM_PER_METER = 100.0;
const double METERS_PER_FOOT = INCHES_PER_FOOT * CM_PER_INCH / CM_PER_METER; // = 0.3048
const double MMETERS_PER_KM =	1000000.0;
const double MMETERS_PER_MILE =	1609344.0;
const double METERS_PER_MILE =	MMETERS_PER_MILE / 1000.0;
const long   SECONDS_PER_MINUTE = 60;
const long   MINUTES_PER_HOUR =	60;
const long   SECONDS_PER_HOUR =	SECONDS_PER_MINUTE * MINUTES_PER_HOUR;
const double RADIANS_PER_DEGREE = M_PI/180.0;
const double DEGREES_PER_RADIAN = 180.0/M_PI;

/** Useful constants **/
const double TWOPI = 2.0 * M_PI;
const double HALFPI = M_PI / 2.0;


/** convert between millimeters per second and miles per hour */
static inline double mmps2mph(double mm)
{
  return mm * SECONDS_PER_HOUR / MMETERS_PER_MILE;
}

static inline double kmph2mmps(double kmph)
{
  return kmph * MMETERS_PER_KM / SECONDS_PER_HOUR;
}

static inline double mph2mmps(double mph)
{
  return mph * MMETERS_PER_MILE / SECONDS_PER_HOUR;
}

/** convert between meters per second and miles per hour */
static inline double mph2mps(double mph)
{
  return mph * METERS_PER_MILE / SECONDS_PER_HOUR;
}

/** convert from meters per second to miles per hour  */
static inline double mps2mph(double mps)
{
  return mps * SECONDS_PER_HOUR / METERS_PER_MILE;
}

/** convert from feet to meters */
static inline double feet2meters(double feet)
{
  return feet * METERS_PER_FOOT;
}

/** convert from meters to feet */
static inline double meters2feet(double meters)
{
  return meters / METERS_PER_FOOT;
}

/** convert timeval to seconds */
//static inline double tv2secs(struct timeval *tv)
//{
//  return tv->tv_sec + (tv->tv_usec / 1000000.0);
//}

/** convert analog input data to corresponding voltage */
static inline double analog_volts(int data, double maxvolts, int nbits)
{
  // clamp value to specified bit range
  int limit = (1<<nbits);
  data &= (limit - 1);
  return (maxvolts * data) / limit;
}

/** convert analog voltage corresponding digital encoding */
/*static inline int analog_to_digital(double voltage,
                                    double maxvolts, int nbits)
{
  return (int) rintf((voltage / maxvolts) * (1<<nbits));
}
*/
namespace UTM
{
  // Grid granularity for rounding UTM coordinates to generate MapXY.
  const double grid_size = 100000.0;    // 100 km grid

// WGS84 Parameters
#define WGS84_A		6378137.0		// major axis
#define WGS84_B		6356752.31424518	// minor axis
#define WGS84_F		0.0033528107		// ellipsoid flattening
#define WGS84_E		0.0818191908		// first eccentricity
#define WGS84_EP	0.0820944379		// second eccentricity

// UTM Parameters
#define UTM_K0		0.9996			// scale factor
#define UTM_FE		500000.0		// false easting
#define UTM_FN_N	0.0           // false northing, northern hemisphere
#define UTM_FN_S	10000000.0    // false northing, southern hemisphere
#define UTM_E2		(WGS84_E*WGS84_E)	// e^2
#define UTM_E4		(UTM_E2*UTM_E2)		// e^4
#define UTM_E6		(UTM_E4*UTM_E2)		// e^6
#define UTM_EP2		(UTM_E2/(1-UTM_E2))	// e'^2

/**
 * Utility function to convert geodetic to UTM position
 * 
 * Units in are floating point degrees (sign for east/west)
 *
 * Units out are meters
 *
 * @todo deprecate this interface in favor of LLtoUTM()
 */
static inline void UTM(double lat, double lon, double *x, double *y)
{
  // constants
  const static double m0 = (1 - UTM_E2/4 - 3*UTM_E4/64 - 5*UTM_E6/256);
  const static double m1 = -(3*UTM_E2/8 + 3*UTM_E4/32 + 45*UTM_E6/1024);
  const static double m2 = (15*UTM_E4/256 + 45*UTM_E6/1024);
  const static double m3 = -(35*UTM_E6/3072);

  // compute the central meridian
  int cm = ((lon >= 0.0)
	    ? ((int)lon - ((int)lon)%6 + 3)
	    : ((int)lon - ((int)lon)%6 - 3));

  // convert degrees into radians
  double rlat = lat * RADIANS_PER_DEGREE;
  double rlon = lon * RADIANS_PER_DEGREE;
  double rlon0 = cm * RADIANS_PER_DEGREE;

  // compute trigonometric functions
  double slat = sin(rlat);
  double clat = cos(rlat);
  double tlat = tan(rlat);

  // decide the false northing at origin
  double fn = (lat > 0) ? UTM_FN_N : UTM_FN_S;

  double T = tlat * tlat;
  double C = UTM_EP2 * clat * clat;
  double A = (rlon - rlon0) * clat;
  double M = WGS84_A * (m0*rlat + m1*sin(2*rlat)
			+ m2*sin(4*rlat) + m3*sin(6*rlat));
  double V = WGS84_A / sqrt(1 - UTM_E2*slat*slat);

  // compute the easting-northing coordinates
  *x = UTM_FE + UTM_K0 * V * (A + (1-T+C)*pow(A,3)/6
			      + (5-18*T+T*T+72*C-58*UTM_EP2)*pow(A,5)/120);
  *y = fn + UTM_K0 * (M + V * tlat * (A*A/2
				      + (5-T+9*C+4*C*C)*pow(A,4)/24
				      + ((61-58*T+T*T+600*C-330*UTM_EP2)
					 * pow(A,6)/720)));

  return;
}


/**
 * Determine the correct UTM letter designator for the
 * given latitude
 *
 * @returns 'Z' if latitude is outside the UTM limits of 84N to 80S
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 */
static inline char UTMLetterDesignator(double Lat)
{
	char LetterDesignator;

	if     ((84 >= Lat) && (Lat >= 72))  LetterDesignator = 'X';
	else if ((72 > Lat) && (Lat >= 64))  LetterDesignator = 'W';
	else if ((64 > Lat) && (Lat >= 56))  LetterDesignator = 'V';
	else if ((56 > Lat) && (Lat >= 48))  LetterDesignator = 'U';
	else if ((48 > Lat) && (Lat >= 40))  LetterDesignator = 'T';
	else if ((40 > Lat) && (Lat >= 32))  LetterDesignator = 'S';
	else if ((32 > Lat) && (Lat >= 24))  LetterDesignator = 'R';
	else if ((24 > Lat) && (Lat >= 16))  LetterDesignator = 'Q';
	else if ((16 > Lat) && (Lat >= 8))   LetterDesignator = 'P';
	else if (( 8 > Lat) && (Lat >= 0))   LetterDesignator = 'N';
	else if (( 0 > Lat) && (Lat >= -8))  LetterDesignator = 'M';
	else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
	else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
	else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
	else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
	else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
	else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
	else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
	else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
	else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
        // 'Z' is an error flag, the Latitude is outside the UTM limits
	else LetterDesignator = 'Z';
	return LetterDesignator;
}

/**
 * Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532 
 *
 * East Longitudes are positive, West longitudes are negative. 
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 */
static inline void LLtoUTM(const double Lat, const double Long, 
                           double &UTMNorthing, double &UTMEasting,
                           char* UTMZone)
{
    //printf("jojo\n\n\n\n");
	double a = WGS84_A;
	double eccSquared = UTM_E2;
	double k0 = UTM_K0;

	double LongOrigin;
	double eccPrimeSquared;
	double N, T, C, A, M;
	
        //Make sure the longitude is between -180.00 .. 179.9
	double LongTemp = (Long+180)-int((Long+180)/360)*360-180;

	double LatRad = Lat*RADIANS_PER_DEGREE;
	double LongRad = LongTemp*RADIANS_PER_DEGREE;
	double LongOriginRad;
	int    ZoneNumber;
 //printf("jojo2\n\n\n\n");
	ZoneNumber = int((LongTemp + 180)/6) + 1;
  
	if( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
		ZoneNumber = 32;
 //printf("jojo3\n\n\n\n");
        // Special zones for Svalbard
	if( Lat >= 72.0 && Lat < 84.0 ) 
	{
	  if(      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
	  else if( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
	  else if( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
	  else if( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
	 }
        // +3 puts origin in middle of zone
	LongOrigin = (ZoneNumber - 1)*6 - 180 + 3; 
	LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;
//printf("jojo4\n\n\n\n");
	//compute the UTM Zone from the latitude and longitude
	sprintf(UTMZone, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));
//printf("jojo5\n\n\n\n");
	eccPrimeSquared = (eccSquared)/(1-eccSquared);

	N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
	T = tan(LatRad)*tan(LatRad);
	C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
	A = cos(LatRad)*(LongRad-LongOriginRad);

	M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64
                - 5*eccSquared*eccSquared*eccSquared/256) * LatRad 
               - (3*eccSquared/8 + 3*eccSquared*eccSquared/32
                  + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
               + (15*eccSquared*eccSquared/256
                  + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad) 
               - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));
	
	UTMEasting = (double)
          (k0*N*(A+(1-T+C)*A*A*A/6
                 + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
           + 500000.0);

	UTMNorthing = (double)
          (k0*(M+N*tan(LatRad)
               *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                 + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));

	if(Lat < 0)
          {
            //10000000 meter offset for southern hemisphere
            UTMNorthing += 10000000.0;
          }
}

/**
 * Converts UTM coords to lat/long.  Equations from USGS Bulletin 1532 
 *
 * East Longitudes are positive, West longitudes are negative. 
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees. 
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 */
static inline void UTMtoLL(float UTMNorthing, float UTMEasting,
                           const char* UTMZone, float& Lat,  float& Long )
{
	double k0 = UTM_K0;
	double a = WGS84_A;
	double eccSquared = UTM_E2;
	double eccPrimeSquared;
	double e1 = (1-sqrt(1-eccSquared))/(1+sqrt(1-eccSquared));
	double N1, T1, C1, R1, D, M;
	double LongOrigin;
	double mu, phi1, phi1Rad;
	double x, y;
	int ZoneNumber;
	char* ZoneLetter;
	int NorthernHemisphere; //1 for northern hemispher, 0 for southern

	x = UTMEasting - 500000.0; //remove 500,000 meter offset for longitude
	y = UTMNorthing;

	ZoneNumber = strtoul(UTMZone, &ZoneLetter, 10);
	if((*ZoneLetter - 'N') >= 0)
		NorthernHemisphere = 1;//point is in northern hemisphere
	else
	{
                //point is in southern hemisphere
		NorthernHemisphere = 0;
                //remove 10,000,000 meter offset used for southern hemisphere
		y -= 10000000.0;
	}

        //+3 puts origin in middle of zone
	LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;
	eccPrimeSquared = (eccSquared)/(1-eccSquared);

	M = y / k0;
	mu = M/(a*(1-eccSquared/4-3*eccSquared*eccSquared/64
                   -5*eccSquared*eccSquared*eccSquared/256));

	phi1Rad = mu + ((3*e1/2-27*e1*e1*e1/32)*sin(2*mu) 
                        + (21*e1*e1/16-55*e1*e1*e1*e1/32)*sin(4*mu)
                        + (151*e1*e1*e1/96)*sin(6*mu));
	phi1 = phi1Rad * DEGREES_PER_RADIAN;

	N1 = a/sqrt(1-eccSquared*sin(phi1Rad)*sin(phi1Rad));
	T1 = tan(phi1Rad)*tan(phi1Rad);
	C1 = eccPrimeSquared*cos(phi1Rad)*cos(phi1Rad);
	R1 = a*(1-eccSquared)/pow(1-eccSquared*sin(phi1Rad)*sin(phi1Rad), 1.5);
	D = x/(N1*k0);

	Lat = phi1Rad - ((N1*tan(phi1Rad)/R1)
                         *(D*D/2
                           -(5+3*T1+10*C1-4*C1*C1-9*eccPrimeSquared)*D*D*D*D/24
                           +(61+90*T1+298*C1+45*T1*T1-252*eccPrimeSquared
                             -3*C1*C1)*D*D*D*D*D*D/720));

	Lat = Lat * DEGREES_PER_RADIAN;

	Long = ((D-(1+2*T1+C1)*D*D*D/6
                 +(5-2*C1+28*T1-3*C1*C1+8*eccPrimeSquared+24*T1*T1)
                 *D*D*D*D*D/120)
                / cos(phi1Rad));
	Long = LongOrigin + Long * DEGREES_PER_RADIAN;

}
} // end namespace UTM

#endif // _UTM_H
