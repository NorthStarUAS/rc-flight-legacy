#include "raw_sat.h"

#include <iostream>
using std::cout;
using std::endl;

// Constants
const double EarthRadius = 6378137.0;        // earth semi-major axis radius (m)
const double ECC2 = 0.0066943799901;         // major eccentricity squared

// Constants for tightly coupled EKF
const double MU = 3.986005e14; //m^3 / sec^2
const double OMEGA_DOT_EARTH = 7.2921151467e-5; // rad/sec
const double c = 299792458; // Speed of light in m/s
const double J2 = 1.082627e-3; // WGS84 Earth's second gravitational constant

// Constants that are no longer used
const double Eccentricity = 0.0818191908426; // major eccentricity of earth ellipsoid

// Skew symmetric matrix from a given vector w
Matrix3d Skew(Vector3d w) {
  Matrix3d C;

  C(0,0) =  0.0;	C(0,1) = -w(2);	C(0,2) =  w(1);
  C(1,0) =  w(2);	C(1,1) =  0.0;	C(1,2) = -w(0);
  C(2,0) = -w(1);	C(2,1) =  w(0);	C(2,2) =  0.0;

  return C;
}

// Calculate the Latitude, Longitude and Altitude given
// the ECEF Coordinates.
Vector3d E2D( Vector3d p_E ) {
    const double Squash = 0.9966471893352525192801545;
    const double ra2 = 1.0 / (EarthRadius * EarthRadius);
    const double e2 = fabs(1 - Squash * Squash);
    const double e4 = e2 * e2;

    // according to
    // H. Vermeille,
    // Direct transformation from geocentric to geodetic ccordinates,
    // Journal of Geodesy (2002) 76:451-454
    Vector3d p_D;
    double X = p_E(0);
    double Y = p_E(1);
    double Z = p_E(2);

    double XXpYY = X*X+Y*Y;
    if( XXpYY + Z*Z < 25 ) {
    	// This function fails near the geocenter region, so catch
    	// that special case here.  Define the innermost sphere of
    	// small radius as earth center and return the coordinates
    	// 0/0/-EQURAD. It may be any other place on geoide's surface,
    	// the Northpole, Hawaii or Wentorf. This one was easy to code
    	// ;-)
    	p_D(0) = 0.0;
    	p_D(1) = 0.0;
    	p_D(2) = -EarthRadius;
    	return p_D;
    }

    double sqrtXXpYY = sqrt(XXpYY);
    double p = XXpYY*ra2;
    double q = Z*Z*(1-e2)*ra2;
    double r = 1/6.0*(p+q-e4);
    double s = e4*p*q/(4*r*r*r);

    /*
       s*(2+s) is negative for s = [-2..0]
       slightly negative values for s due to floating point rounding errors
       cause nan for sqrt(s*(2+s))
       We can probably clamp the resulting parable to positive numbers
    */
    if( s >= -2.0 && s <= 0.0 ) {
      s = 0.0;
    }

    double t = pow(1+s+sqrt(s*(2+s)), 1/3.0);
    double u = r*(1+t+1/t);
    double v = sqrt(u*u+e4*q);
    double w = e2*(u+v-q)/(2*v);
    double k = sqrt(u+v+w*w)-w;
    double D = k*sqrtXXpYY/(k+e2);
    double sqrtDDpZZ = sqrt(D*D+Z*Z);

    p_D(1) = 2*atan2(Y, X+sqrtXXpYY);
    p_D(0) = 2*atan2(Z, D+sqrtDDpZZ);
    p_D(2) = (k+e2-1)*sqrtDDpZZ/k;

    return p_D;
}

// EphemerisData (subframe1,2,3) to Satellite ecef x, y, z in meter, vx, vy, vz in m/s
VectorXd oldEphemerisData2Satecef(float t,
                               uint32_t TOW, uint8_t L2, uint16_t week_No, uint8_t L2_Flag, uint8_t SV_Acc, uint8_t SV_Hlth,
                               double T_GD, uint16_t IODC, double t_OC, int8_t a_f2, double a_f1, double a_f0,
                               uint8_t IODE, double C_rs, double delta_n, double M_0, double C_uc, double ecc, double C_us,
                               double sqrt_A, double t_OE, double C_ic, double Omega_0, double C_is, double i_0, double C_rc,
                               double omega, double Omega_dot, double IDOT)
{
    // All the equations are based ON: https://www.gps.gov/technical/icwg/IS-GPS-200H.pdf
    // pg. 104-105, Also Grove  p335-338

    // Process subframe 1,2,3 information
    double A_semiMajorAxis;        // Semi-major axis
    double n_0_computedMeanMotion; // Computed mean motion
    double n_correctedMeanMotion;  // Corrected mean motion
    double e_eccentricity;         // Eccentricity
    //double phi_k_argumentOfLattitude;   // Argument of latitude
    double M_0_trueAnomalyAtRef;
    double omega0_longitudeofAscendingNodeofOrbitPlane;
    double omega_argumentOfPerigee;
    double omegaDot_argumentOfPerigee;
    double i_0_inclinationAtRef;
    double iDot_rateOfInclination;

    A_semiMajorAxis = pow(sqrt_A, 2);
    n_0_computedMeanMotion = sqrt(MU / pow(A_semiMajorAxis, 3));
    n_correctedMeanMotion = n_0_computedMeanMotion + delta_n;
    e_eccentricity = ecc;
    M_0_trueAnomalyAtRef = M_0;
    omega0_longitudeofAscendingNodeofOrbitPlane = Omega_0;
    omega_argumentOfPerigee = omega;
    omegaDot_argumentOfPerigee = Omega_dot;
    i_0_inclinationAtRef = i_0;
    iDot_rateOfInclination = IDOT;

    // Compute the time from the ephemeris reference epoch
    double t_k_timeFromReferenceEpoch = t - t_OE;
    // Correct that time for end-of-week crossovers
    if (t_k_timeFromReferenceEpoch > 302400)
    {
        t_k_timeFromReferenceEpoch -= 604800;
    }
    if (t_k_timeFromReferenceEpoch < -302400)
    {
        t_k_timeFromReferenceEpoch += 604800;
    }

    // Compute the mean anomaly
    double M_k_meanAnomaly = M_0_trueAnomalyAtRef + n_correctedMeanMotion * t_k_timeFromReferenceEpoch;

    // Below, we iteratively solve for E_k_eccentricAnomaly using Newton-Raphson method
    double solutionError = 1000000.;
    double E_k_eccentricAnomaly = 1.;
    double currentDerivative = 0;
    int iterationCount = 0;

    solutionError = (E_k_eccentricAnomaly -
                     (e_eccentricity * sin(E_k_eccentricAnomaly)) -
                     M_k_meanAnomaly);

    while ((fabs(solutionError) > 1.0e-6) &&
           iterationCount < 1000)
    {
        currentDerivative = (1.0 - (e_eccentricity * cos(E_k_eccentricAnomaly)));
        E_k_eccentricAnomaly = E_k_eccentricAnomaly - solutionError / currentDerivative;

        solutionError = (E_k_eccentricAnomaly -
                         (e_eccentricity * sin(E_k_eccentricAnomaly)) -
                         M_k_meanAnomaly);
        iterationCount += 1;
        //   if (VERBOSE)
        //   {
        //     std::cout<< "Iteration #: " << iterationCount << " Error: " << solutionError << std::endl;
        //   }
    }
    double cos_E_k = cos(E_k_eccentricAnomaly);
    double sin_E_k = sin(E_k_eccentricAnomaly);
    double nu_k_trueAnomaly = atan2(
        (sqrt(1.0 - pow(e_eccentricity, 2)) * sin_E_k) /
            (1.0 - (e_eccentricity * cos_E_k)),
        (cos_E_k - e_eccentricity) /
            (1.0 - e_eccentricity * cos_E_k));

    double phi_k_argumentOfLatitude = nu_k_trueAnomaly + omega_argumentOfPerigee;

    // Compute the corrective 2nd order terms
    double sin2PhiK = sin(2.0 * phi_k_argumentOfLatitude);
    double cos2PhiK = cos(2.0 * phi_k_argumentOfLatitude);

    double deltaU_argumentOfLatCorrection = (C_us * sin2PhiK) + (C_uc * cos2PhiK);
    double deltaR_radiusCorrection = (C_rs * sin2PhiK) + (C_rc * cos2PhiK);
    double deltaI_inclinationCorrection = (C_is * sin2PhiK) + (C_ic * cos2PhiK);

    // Now compute the updated corrected orbital elements
    double u_argumentOfLat = phi_k_argumentOfLatitude + deltaU_argumentOfLatCorrection;
    double r_radius = (A_semiMajorAxis * (1.0 - (e_eccentricity * cos_E_k))) + deltaR_radiusCorrection;
    double i_inclination =
        i_0_inclinationAtRef +
        (iDot_rateOfInclination * t_k_timeFromReferenceEpoch) +
        deltaI_inclinationCorrection;

    // Compute the satellite position within the orbital plane
    double xPositionOrbitalPlane = r_radius * cos(u_argumentOfLat);
    double yPositionOrbitalPlane = r_radius * sin(u_argumentOfLat);
    double omegaK_longitudeAscendingNode =
        omega0_longitudeofAscendingNodeofOrbitPlane +
        ((omegaDot_argumentOfPerigee - OMEGA_DOT_EARTH) * t_k_timeFromReferenceEpoch) -
        (OMEGA_DOT_EARTH * t_OE);

    double sinOmegaK = sin(omegaK_longitudeAscendingNode);
    double cosOmegaK = cos(omegaK_longitudeAscendingNode);

    double sinIK = sin(i_inclination);
    double cosIK = cos(i_inclination);
    // Earth-fixed coordinates:
    double x = (xPositionOrbitalPlane * cosOmegaK) - (yPositionOrbitalPlane * cosIK * sinOmegaK);
    double y = (xPositionOrbitalPlane * sinOmegaK) + (yPositionOrbitalPlane * cosIK * cosOmegaK);
    double z = (yPositionOrbitalPlane * sinIK);


    
    // ECEF velocity calculation:
    double E_dot_k_eccentricAnomaly = n_correctedMeanMotion/(1.0 - (e_eccentricity * cos_E_k)); // Eq.(8.21)
    double phi_dot_k_argumentOfLatitude = sin(nu_k_trueAnomaly)/sin_E_k*E_dot_k_eccentricAnomaly; // Eq.(8.22)
    double r_dot_o_os = (A_semiMajorAxis*e_eccentricity*sin_E_k)*E_dot_k_eccentricAnomaly +
                        2*((C_rs * cos2PhiK) - (C_rc * sin2PhiK))*phi_dot_k_argumentOfLatitude; // Eq.(8.23a)
    double u_dot_o_os = (1+2*C_us * cos2PhiK - 2*C_uc * sin2PhiK)*phi_dot_k_argumentOfLatitude; // Eq.(8.23b)                    
     
    double x_dot_o_os = r_dot_o_os*cos(u_argumentOfLat) - r_radius*u_dot_o_os*sin(u_argumentOfLat); // Eq.(8.24a)
    double y_dot_o_os = r_dot_o_os*sin(u_argumentOfLat) + r_radius*u_dot_o_os*cos(u_argumentOfLat); // Eq.(8.24b)

    double omega_dot_K_longitudeAscendingNode = omegaDot_argumentOfPerigee - OMEGA_DOT_EARTH; // Eq. (8.25)
    double i_dot_inclination = iDot_rateOfInclination + 2*((C_is * cos2PhiK) - (C_ic * sin2PhiK))*phi_dot_k_argumentOfLatitude; // Eq. (8.26)
    
    // Eq. (8.27)
    double vx = (x_dot_o_os*cosOmegaK - y_dot_o_os*cosIK*sinOmegaK + i_dot_inclination*yPositionOrbitalPlane*sinIK*sinOmegaK) -
                omega_dot_K_longitudeAscendingNode*(xPositionOrbitalPlane*sinOmegaK + yPositionOrbitalPlane*cosIK*cosOmegaK);
    double vy = (x_dot_o_os*sinOmegaK + y_dot_o_os*cosIK*cosOmegaK - i_dot_inclination*yPositionOrbitalPlane*sinIK*cosOmegaK) -
                omega_dot_K_longitudeAscendingNode*(-xPositionOrbitalPlane*cosOmegaK +yPositionOrbitalPlane*cosIK*sinOmegaK);
    double vz = (y_dot_o_os*sinIK + i_dot_inclination*yPositionOrbitalPlane*cosIK);
    
    VectorXd pos_vel_Sat_ecef(6);
    pos_vel_Sat_ecef << x, y, z, vx, vy, vz;
    // cout << x << y << y << endl;

    return pos_vel_Sat_ecef;
}

// process raw measurement to give a 8 x 1 matrix (range, range rate, x,y,z,vx,vy,vz)
VectorXd EphemerisData2PosVelClock(GNSS_raw_measurement gnss_raw_measurement)
{
    // All the equations are based ON: https://www.gps.gov/technical/icwg/IS-GPS-200H.pdf
    // pg. 104-105, Also Grove  p335-338

    // Process subframe 1,2,3 information
    printf("  sqrta: %.8f\n", gnss_raw_measurement.sqrtA);
    double A_semiMajorAxis = gnss_raw_measurement.sqrtA*gnss_raw_measurement.sqrtA;        // Semi-major axis
    printf("  a: %.8f\n", A_semiMajorAxis);
    double n_0_computedMeanMotion = sqrt(MU / (A_semiMajorAxis*A_semiMajorAxis*A_semiMajorAxis)); // Computed mean motion
    double n_correctedMeanMotion = n_0_computedMeanMotion + gnss_raw_measurement.deltan;// Corrected mean motion
    printf("  ma_dot: %.20f\n", n_correctedMeanMotion);
    double e_eccentricity = gnss_raw_measurement.e; // Eccentricity
    //double phi_k_argumentOfLattitude;   // Argument of latitude
    double M_0_trueAnomalyAtRef = gnss_raw_measurement.M0;
    double omega0_longitudeofAscendingNodeofOrbitPlane = gnss_raw_measurement.Omega0;
    double omega_argumentOfPerigee = gnss_raw_measurement.omega;
    double omegaDot_argumentOfPerigee = gnss_raw_measurement.Omegad;
    double i_0_inclinationAtRef = gnss_raw_measurement.i0;
    double iDot_rateOfInclination = gnss_raw_measurement.IDOT;

    double t_OE = gnss_raw_measurement.toe;
    // 2nd harmonic terms
    double C_us = gnss_raw_measurement.Cus;
    double C_uc = gnss_raw_measurement.Cuc;
    double C_rs = gnss_raw_measurement.Crs;
    double C_rc = gnss_raw_measurement.Crc;
    double C_is = gnss_raw_measurement.Cis;
    double C_ic = gnss_raw_measurement.Cic;

    // Compute the time from the ephemeris reference epoch
    double t_k_timeFromReferenceEpoch = gnss_raw_measurement.timestamp - t_OE;
    printf("  time: %.2f toe: %.2f tdiff: %.2f\n", gnss_raw_measurement.timestamp, t_OE, t_k_timeFromReferenceEpoch);
    // Correct that time for end-of-week crossovers
    if (t_k_timeFromReferenceEpoch > 302400)
    {
        t_k_timeFromReferenceEpoch -= 604800;
    }
    if (t_k_timeFromReferenceEpoch < -302400)
    {
        t_k_timeFromReferenceEpoch += 604800;
    }

    // Compute the mean anomaly
    double M_k_meanAnomaly = M_0_trueAnomalyAtRef + n_correctedMeanMotion * t_k_timeFromReferenceEpoch;
    printf("  m0: %.20f\n", M_0_trueAnomalyAtRef);
    printf("  ma: %.20f\n", M_k_meanAnomaly);

    // Below, we iteratively solve for E_k_eccentricAnomaly using Newton-Raphson method
    double solutionError = 1000000.;
    double E_k_eccentricAnomaly = 1.;
    double currentDerivative = 0;
    int iterationCount = 0;

    solutionError = (E_k_eccentricAnomaly -
                     (e_eccentricity * sin(E_k_eccentricAnomaly)) -
                     M_k_meanAnomaly);

    while ((fabs(solutionError) > 1.0e-6) &&
           iterationCount < 1000)
    {
        currentDerivative = (1.0 - (e_eccentricity * cos(E_k_eccentricAnomaly)));
        E_k_eccentricAnomaly = E_k_eccentricAnomaly - solutionError / currentDerivative;

        solutionError = (E_k_eccentricAnomaly -
                         (e_eccentricity * sin(E_k_eccentricAnomaly)) -
                         M_k_meanAnomaly);
        iterationCount += 1;
        //   if (VERBOSE)
        //   {
        //     std::cout<< "Iteration #: " << iterationCount << " Error: " << solutionError << std::endl;
        //   }
    }
    printf("  ea: %.20f\n", E_k_eccentricAnomaly);
    double cos_E_k = cos(E_k_eccentricAnomaly);
    double sin_E_k = sin(E_k_eccentricAnomaly);
    double nu_k_trueAnomaly = atan2(
        (sqrt(1.0 - e_eccentricity*e_eccentricity) * sin_E_k) /
            (1.0 - (e_eccentricity * cos_E_k)),
        (cos_E_k - e_eccentricity) /
            (1.0 - e_eccentricity * cos_E_k));

    double phi_k_argumentOfLatitude = nu_k_trueAnomaly + omega_argumentOfPerigee;
    printf("  al: %.20f\n", phi_k_argumentOfLatitude);
 
    // Compute the corrective 2nd order terms
    double sin2PhiK = sin(2.0 * phi_k_argumentOfLatitude);
    double cos2PhiK = cos(2.0 * phi_k_argumentOfLatitude);

    double deltaU_argumentOfLatCorrection = (C_us * sin2PhiK) + (C_uc * cos2PhiK);
    double deltaR_radiusCorrection = (C_rs * sin2PhiK) + (C_rc * cos2PhiK);
    double deltaI_inclinationCorrection = (C_is * sin2PhiK) + (C_ic * cos2PhiK);

    // Now compute the updated corrected orbital elements
    double u_argumentOfLat = phi_k_argumentOfLatitude + deltaU_argumentOfLatCorrection;
    printf("  cal: %.20f\n", u_argumentOfLat);
    double r_radius = (A_semiMajorAxis * (1.0 - (e_eccentricity * cos_E_k))) + deltaR_radiusCorrection;
    printf("  r: %.20f\n", r_radius);
    double i_inclination =
        i_0_inclinationAtRef +
        (iDot_rateOfInclination * t_k_timeFromReferenceEpoch) +
        deltaI_inclinationCorrection;
    printf("  inc: %.20f\n", i_inclination);

    // Compute the satellite position within the orbital plane
    double xPositionOrbitalPlane = r_radius * cos(u_argumentOfLat);
    double yPositionOrbitalPlane = r_radius * sin(u_argumentOfLat);
    printf("  x: %.20f y: %.20f\n", xPositionOrbitalPlane, yPositionOrbitalPlane);
    double omegaK_longitudeAscendingNode =
        omega0_longitudeofAscendingNodeofOrbitPlane +
        ((omegaDot_argumentOfPerigee - OMEGA_DOT_EARTH) * t_k_timeFromReferenceEpoch) -
        (OMEGA_DOT_EARTH * t_OE);
    printf("  om: %.20f\n", omegaK_longitudeAscendingNode);

    double sinOmegaK = sin(omegaK_longitudeAscendingNode);
    double cosOmegaK = cos(omegaK_longitudeAscendingNode);

    double sinIK = sin(i_inclination);
    double cosIK = cos(i_inclination);
    // Earth-fixed coordinates:
    double x = (xPositionOrbitalPlane * cosOmegaK) - (yPositionOrbitalPlane * cosIK * sinOmegaK);
    double y = (xPositionOrbitalPlane * sinOmegaK) + (yPositionOrbitalPlane * cosIK * cosOmegaK);
    double z = (yPositionOrbitalPlane * sinIK);
    printf("  xyz: %.2f %.2f %.2f\n", x, y, z);

    // ECEF velocity calculation:
    double E_dot_k_eccentricAnomaly = n_correctedMeanMotion / (1.0 - (e_eccentricity * cos_E_k));     // Eq.(8.21)
    double phi_dot_k_argumentOfLatitude = sin(nu_k_trueAnomaly) / sin_E_k * E_dot_k_eccentricAnomaly; // Eq.(8.22)
    double r_dot_o_os = (A_semiMajorAxis * e_eccentricity * sin_E_k) * E_dot_k_eccentricAnomaly +
                        2 * ((C_rs * cos2PhiK) - (C_rc * sin2PhiK)) * phi_dot_k_argumentOfLatitude;     // Eq.(8.23a)
    double u_dot_o_os = (1 + 2 * C_us * cos2PhiK - 2 * C_uc * sin2PhiK) * phi_dot_k_argumentOfLatitude; // Eq.(8.23b)

    double x_dot_o_os = r_dot_o_os * cos(u_argumentOfLat) - r_radius * u_dot_o_os * sin(u_argumentOfLat); // Eq.(8.24a)
    double y_dot_o_os = r_dot_o_os * sin(u_argumentOfLat) + r_radius * u_dot_o_os * cos(u_argumentOfLat); // Eq.(8.24b)

    double omega_dot_K_longitudeAscendingNode = omegaDot_argumentOfPerigee - OMEGA_DOT_EARTH;                                       // Eq. (8.25)
    double i_dot_inclination = iDot_rateOfInclination + 2 * ((C_is * cos2PhiK) - (C_ic * sin2PhiK)) * phi_dot_k_argumentOfLatitude; // Eq. (8.26)

    // Eq. (8.27)
    double vx = (x_dot_o_os * cosOmegaK - y_dot_o_os * cosIK * sinOmegaK + i_dot_inclination * yPositionOrbitalPlane * sinIK * sinOmegaK) -
                omega_dot_K_longitudeAscendingNode * (xPositionOrbitalPlane * sinOmegaK + yPositionOrbitalPlane * cosIK * cosOmegaK);
    double vy = (x_dot_o_os * sinOmegaK + y_dot_o_os * cosIK * cosOmegaK - i_dot_inclination * yPositionOrbitalPlane * sinIK * cosOmegaK) -
                omega_dot_K_longitudeAscendingNode * (-xPositionOrbitalPlane * cosOmegaK + yPositionOrbitalPlane * cosIK * sinOmegaK);
    double vz = (y_dot_o_os * sinIK + i_dot_inclination * yPositionOrbitalPlane * cosIK);

    // Estimate satellite clock error
    // relativistic correction term, seconds
    const double GPS_F = -4.442807633e-10;
    double deltatr = GPS_F * gnss_raw_measurement.e * gnss_raw_measurement.sqrtA * sin_E_k;
    // toff == SV PRN code phase offset, seconds
    // we want tgps, but our t (tsv) is close enough.
    double toff = gnss_raw_measurement.timestamp - gnss_raw_measurement.toc;
    // handle week rollover
    if (toff > 302400) {
        toff -= 604800;
    }
    if (toff < -302400) {
        toff += 604800;
    }
    // SV Clock Correction
    double deltatsv = (gnss_raw_measurement.af0
                       + gnss_raw_measurement.af1*toff +
                       gnss_raw_measurement.af2*toff*toff + deltatr);
    printf("sv %d: toc: %.8f toff: %.8f %.2f m deltatsv = %.f sec\n",
	   gnss_raw_measurement.gnssid, gnss_raw_measurement.toc, toff, deltatsv*c, deltatsv);
    /* printf("deltatr %e deltatsv %e Tgd %.e\n"
           "URE %s Health %s", deltatr, deltatsv, ephm['Tgd'], ura2ure[ephm['ura']],
           health_str[ephm['hlth'] & 0x1f])) */
                      
    VectorXd pos_vel_ecef_clock(8);
    double lambda = (2 * c) / 1575.4282e6;  // L1 according ublox8
    // https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrPrtSpec_%28UBX-13003221%29.pdf
    double PseudorangeRate = lambda * gnss_raw_measurement.doppler;
    pos_vel_ecef_clock << gnss_raw_measurement.pseudorange + deltatsv*c, PseudorangeRate, x, y, z, vx, vy, vz;
    // cout << x << y << y << endl;

    return pos_vel_ecef_clock;
}

// compute vehicle's postion, velocity in E frame and clock offset (m) and drift (m/s) 
void GNSS_LS_pos_vel(MatrixXd &gnss_measurement,
                     Vector3d &pEst_E_m_, Vector3d &vEst_E_mps_, double &clockBias_m_, double &clockRateBias_mps_)
{
    /*
   GNSS_LS_position_velocity - Calculates position, velocity, clock offset, 
   and clock drift using unweighted iterated least squares. Separate
   calculations are implemented for position and clock offset and for
   velocity and clock drift

    % Inputs:
    %   GNSS_measurements     GNSS measurement data:
    %     Column 0              Pseudo-range measurements (m)
    %     Column 1              Pseudo-range rate measurements (m/s)
    %     Columns 2-4           Satellite ECEF position (m)
    %     Columns 5-7           Satellite ECEF velocity (m/s)
    %   no_GNSS_meas          Number of satellites for which measurements are
    %                         supplied
    %   pEst_E_m_        prior predicted ECEF user position (m)
    %   vEst_E_mps_      prior predicted ECEF user velocity (m/s)
    %
    % Outputs:

        output is a 8 by 1 matrix containing the following: 

    %   est_r_ea_e            estimated ECEF user position (m)
    %   est_v_ea_e            estimated ECEF user velocity (m/s)
    %   est_clock             estimated receiver clock offset (m) and drift (m/s)
   */

    int no_sat = gnss_measurement.rows();
    cout << "no_sats: " << no_sat << endl;
    cout << "pEst_E_m_:" << pEst_E_m_.transpose() << endl;
    cout << "vEst_E_mps_:" << vEst_E_mps_.transpose() << endl; // questionable vEst, check GNSS_LS_pos_vel calc later 
    cout << "clockBias_m: " << clockBias_m_ << endl;
    cout << "clockRateBias_mps " <<  clockRateBias_mps_ << endl;
  
    // Position and Clock OFFSET
    VectorXd x_pred(4, 1);
    x_pred.setZero();
    x_pred.segment(0, 3) = pEst_E_m_;
    x_pred(3) = 0;

    // allocate space for estimation
    VectorXd delta_r(3, 1);
    delta_r.setZero();
    double approx_range = 0;
    double range = 0;
    double range_rate = 0;
    VectorXd pred_meas(no_sat, 1);
    pred_meas.setZero();
    MatrixXd H(no_sat, 4);
    H.setZero();
    Matrix<double, 3, 3> T_E2I;
    T_E2I.setIdentity();
    VectorXd u_as_E(3, 1);
    u_as_E.setZero();

    // output
    Matrix<double, 4, 1> x_est_1;
    x_est_1.setZero();
    Matrix<double, 4, 1> x_est_2;
    x_est_2.setZero();
    VectorXd output(8, 1);
    output.setZero();

    // set the flag for the while-loop
    double test_convergence = 1;
    while (test_convergence > 0.0001)
    {
        cout << "test_convergence: " << test_convergence << endl;
        
        for (int j = 0; j < no_sat; j++)
        {
            Vector3d x_temp;
            x_temp(0) = gnss_measurement(j, 2);
            x_temp(1) = gnss_measurement(j, 3);
            x_temp(2) = gnss_measurement(j, 4);

            delta_r = x_temp - x_pred.segment(0, 3);
            approx_range = delta_r.norm();

            // Calculate frame rotation during signal transit time using (Grove2nd:8.36)
            T_E2I(0, 0) = 1;
            T_E2I(0, 1) = OMEGA_DOT_EARTH * approx_range / c;
            T_E2I(0, 2) = 0;
            T_E2I(1, 0) = -OMEGA_DOT_EARTH * approx_range / c;
            T_E2I(1, 1) = 1;
            T_E2I(1, 2) = 0;
            T_E2I(2, 0) = 0;
            T_E2I(2, 1) = 0;
            T_E2I(2, 2) = 1;

            // same thing without the trig approximations
            // double wEtau = OMEGA_DOT_EARTH * approx_range / c;
            // T_E2I(0, 0) = cos(wEtau);
            // T_E2I(0, 1) = sin(wEtau);
            // T_E2I(0, 2) = 0;
            // T_E2I(1, 0) = -sin(wEtau);
            // T_E2I(1, 1) = cos(wEtau);
            // T_E2I(1, 2) = 0;
            // T_E2I(2, 0) = 0;
            // T_E2I(2, 1) = 0;
            // T_E2I(2, 2) = 1;

            delta_r = T_E2I.cast<double>() * x_temp - x_pred.segment(0, 3);
            range = delta_r.norm();
            pred_meas(j, 0) = range + x_pred(3);
            u_as_E = delta_r / range;
            H(j, 0) = -u_as_E(0);
            H(j, 1) = -u_as_E(1);
            H(j, 2) = -u_as_E(2);
            H(j, 3) = 1;
        }
        // least sqaure method
        x_est_1 = x_pred + (H.transpose() * H).inverse() * H.transpose() * (gnss_measurement.col(0) - pred_meas);
        test_convergence = (x_est_1 - x_pred).norm();
        x_pred = x_est_1;
        cout << "x_pred: " << x_pred << endl;
    }
    
    // Earth rotation vector and matrix
    Vector3d omega_ie; // Earth rate vector
    Matrix3d Omega_ie; // Earth rate rotation matrix
    omega_ie(0) = 0.0;
    omega_ie(1) = 0.0;
    omega_ie(2) = OMEGA_DOT_EARTH;
    Omega_ie = Skew(omega_ie);

    // save the estimated postion and clock offset
    // output(0) = x_est_1(0);
    // output(1) = x_est_1(1);
    // output(2) = x_est_1(2);
    // output(6) = x_est_1(3); // record clock offset
    pEst_E_m_(0) = x_est_1(0);
    pEst_E_m_(1) = x_est_1(1);
    pEst_E_m_(2) = x_est_1(2);
    clockBias_m_ = (float)x_est_1(3); // record clock offset

    x_pred.setZero();
    pred_meas.setZero();
    x_pred.segment(0, 3) = vEst_E_mps_;
    x_pred(3) = 0;
    test_convergence = 1;
    u_as_E.setZero();
    
    while (test_convergence > 0.0001)
    {
        for (int j = 0; j < no_sat; j++)
        {
            Vector3d p_temp, v_temp;
            p_temp(0) = gnss_measurement(j, 2);
            p_temp(1) = gnss_measurement(j, 3);
            p_temp(2) = gnss_measurement(j, 4);
            v_temp(0) = gnss_measurement(j, 5);
            v_temp(1) = gnss_measurement(j, 6);
            v_temp(2) = gnss_measurement(j, 7);

            delta_r = p_temp - x_est_1.segment(0, 3);
            approx_range = delta_r.norm();

            // Calculate frame rotation during signal transit time using (Grove2nd:8.36)
            T_E2I(0, 0) = 1;
            T_E2I(0, 1) = OMEGA_DOT_EARTH * approx_range / c;
            T_E2I(0, 2) = 0;
            T_E2I(1, 0) = -OMEGA_DOT_EARTH * approx_range / c;
            T_E2I(1, 1) = 1;
            T_E2I(1, 2) = 0;
            T_E2I(2, 0) = 0;
            T_E2I(2, 1) = 0;
            T_E2I(2, 2) = 1;

            delta_r = T_E2I.cast<double>() * p_temp - x_est_1.segment(0, 3);
            range = delta_r.norm();
            u_as_E = delta_r / range;
     
            // Predict pseudo-range rate using (9.165)
            range_rate = u_as_E.transpose() * (T_E2I.cast<double>() * (v_temp + Omega_ie.cast<double>() * p_temp) -
                                               (x_pred.segment(0, 3) + Omega_ie.cast<double>() * x_est_1.segment(0, 3)));
       
            pred_meas(j, 0) = range_rate + x_pred(3);  
    
            H(j, 0) = -u_as_E(0);
            H(j, 1) = -u_as_E(1);
            H(j, 2) = -u_as_E(2);
            H(j, 3) = 1;
        }

        x_est_2 = x_pred + (H.transpose() * H).inverse() * H.transpose() * (gnss_measurement.col(1) - pred_meas);
        test_convergence = (x_est_2 - x_pred).norm();
        x_pred = x_est_2;
    }
    
    // save the estimated postion and clock offset
    // output(3) = x_est_2(0);
    // output(4) = x_est_2(1);
    // output(5) = x_est_2(2);
    // output(7) = x_est_2(3);
    vEst_E_mps_(0) = x_est_2(0);
    vEst_E_mps_(1) = x_est_2(1);
    vEst_E_mps_(2) = x_est_2(2);
    clockRateBias_mps_ = (float)x_est_2(3);
}

// if we know our position, try to find a clock bias that minimizes the difference between pseudorange and geometric range
void GNSS_clock_bias(MatrixXd &gnss_measurement, Vector3d &pos_true)
{
    /*
   GNSS_LS_position_velocity - Calculates position, velocity, clock offset, 
   and clock drift using unweighted iterated least squares. Separate
   calculations are implemented for position and clock offset and for
   velocity and clock drift

    % Inputs:
    %   GNSS_measurements     GNSS measurement data:
    %     Column 0              Pseudo-range measurements (m)
    %     Column 1              Pseudo-range rate measurements (m/s)
    %     Columns 2-4           Satellite ECEF position (m)
    %     Columns 5-7           Satellite ECEF velocity (m/s)
    %   no_GNSS_meas          Number of satellites for which measurements are
    %                         supplied
    %   pEst_E_m_        prior predicted ECEF user position (m)
    %   vEst_E_mps_      prior predicted ECEF user velocity (m/s)
    %
    % Outputs:

        output is a 8 by 1 matrix containing the following: 

    %   est_r_ea_e            estimated ECEF user position (m)
    %   est_v_ea_e            estimated ECEF user velocity (m/s)
    %   est_clock             estimated receiver clock offset (m) and drift (m/s)
   */

    int no_sat = gnss_measurement.rows();
    cout << "no_sats: " << no_sat << endl;
    cout << "pos_true:" << pos_true.transpose() << endl;
  
    // allocate space for estimation
    Matrix<double, 3, 3> T_E2I;
    T_E2I.Identity();

    // output
    Matrix<double, 4, 1> x_est_1;
    x_est_1.setZero();
    Matrix<double, 4, 1> x_est_2;
    x_est_2.setZero();
    VectorXd output(8, 1);
    output.setZero();

    // set the flag for the while-loop
    double cb = -0.1;
    double cdt = 0.01;
    while (cb <= 0.1) {
        cout << "clock bias::" << cb << endl;
        for (int j = 0; j < no_sat; j++) {
            Vector3d x_temp;
            x_temp(0) = gnss_measurement(j, 2);
            x_temp(1) = gnss_measurement(j, 3);
            x_temp(2) = gnss_measurement(j, 4);

            // same thing without the trig approximations
            double wEtau = OMEGA_DOT_EARTH * cb;
            T_E2I(0, 0) = cos(wEtau);
            T_E2I(0, 1) = sin(wEtau);
            T_E2I(0, 2) = 0;
            T_E2I(1, 0) = -sin(wEtau);
            T_E2I(1, 1) = cos(wEtau);
            T_E2I(1, 2) = 0;
            T_E2I(2, 0) = 0;
            T_E2I(2, 1) = 0;
            T_E2I(2, 2) = 1;

            Vector3d new_sat_pos = T_E2I.cast<double>() * x_temp;
            // cout << "  " << x_temp.transpose() << endl;
            Vector3d delta = new_sat_pos - pos_true;
            double geo_range = delta.norm();
            printf("  pr: %.1f geo: %.1f diff: %.1f\n", gnss_measurement(j, 0), geo_range,
                   gnss_measurement(j, 0) - geo_range);
        }
        cb += cdt;
    }
    cout << endl;    
}
