from scipy import io as sio
import numpy as np
import os

"""
Sample script/function for loading .mat files in Python.

Hamid M.
Oct 11, 2013
"""

def load_struct(filepath, print_summary=False):
    """
    Load flight data from UMN AEM UAV Group.
    
    Parameter
    ---------
    filepath : (absolute) path to data file (string).  
    
    Returns
    -------
    flight_data : struct with flight data (mat_struct)
    flight_info : struct with flight info (mat_sruct)
    
    Note
    ----
    struct_as_record flag used to load the MATLAB structs as python
    objects rather than numpy structured arrarys 
    Reference: http://docs.scipy.org/doc/scipy/reference/tutorial/io.html
        
    """
    
    data = sio.loadmat(filepath, struct_as_record=False, squeeze_me=True)
    try:
        flight_data, flight_info = data['flight_data'], data['flight_info']
        #======================================================================
        #============================ ADDED BY ADHIKA =========================
        #======================== TO HANDLE SIMULATED DATA ====================
    except KeyError:
        # If there is no flight_info then this is simulation data.
        # There are more information available here
        flight_data = data['flight_data']
        flight_info = []
        true_data = data['true']
        trueimu = data['imuT']

    # Fix naming of theta as the
    try:
        flight_data.theta
    except AttributeError:
        flight_data.theta = flight_data.the
    
    # Might already have bias, so add them back
    try:
        flight_data.p += flight_data.p_bias
        flight_data.q += flight_data.q_bias
        flight_data.r += flight_data.r_bias

        flight_data.ax += flight_data.ax_bias
        flight_data.ay += flight_data.ay_bias
        flight_data.az += flight_data.az_bias
    except AttributeError:
        pass

    # For simulation, GPS_TOW field and navValid is not available. Make it so
    try:
        flight_data.GPS_TOW
        flight_data.navValid
    except AttributeError:
        idx = np.nonzero(np.isnan(flight_data.lat)==False)
        flight_data.navValid = np.ones(len(flight_data.time))
        flight_data.navValid[idx] = 0
        
        flight_data.GPS_TOW = np.nan*np.zeros(len(flight_data.time))
        flight_data.GPS_TOW[idx] = flight_data.time[idx]

        flight_data.navlat = np.deg2rad(true_data.lat)
        flight_data.navlon = np.deg2rad(true_data.lon)
        flight_data.navalt = true_data.alt

        flight_data.navvn = true_data.vn
        flight_data.navve = true_data.ve
        flight_data.navvd = true_data.vd

        flight_data.ptrue = trueimu.p
        flight_data.qtrue = trueimu.q
        flight_data.rtrue = trueimu.r

        flight_data.axtrue = trueimu.fx
        flight_data.aytrue = trueimu.fy
        flight_data.aztrue = trueimu.fz
        #======================================================================
    #del(data)
    
    if print_summary:
        print('Loaded Data Summary')
        print('-------------------')
        print('* File: %s' % filepath.split(os.path.sep))[-1]
        print('* Date: %s' % flight_info.date)
        print('* Aircraft: %s' % flight_info.aircraft)
    
    return flight_data, flight_info
   
   
if __name__ == '__main__':
    # Example of loading UAV Data 
    filepath = 'faser_flight05_basline_control_ekf15_baseline_validation_08_07_2012.mat'
    uav_flight_data, uav_flight_info = load_struct(filepath, print_summary=True )
    
    # Example of loading Demoz Book Data
    filepath = 'flight_data.mat'
    gnss_flight_data = sio.loadmat(filepath, squeeze_me=True)
    
    print('Data has been loaded from both UAV and GNSS Applications data examples.')
    print('The UAV data is loaded as a `struct` with elements accessible using `.`.')
    print('The GNSS flight data is loaded as a dict with elements accessible using keys.')
