

def complementary_filter_update(prev_filtered_gyro, prev_filtered_mag, omega_meas, B_meas, alpha=0.98):
   
    B_filtered = alpha * B_meas + (1 - alpha) * prev_filtered_mag
    omega_filtered = alpha * omega_meas + (1 - alpha) * prev_filtered_gyro
    

    
    return B_filtered, omega_filtered

