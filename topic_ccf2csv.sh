# save into .csv files
rostopic echo -p /imu/euler_comp_filter > data_filter.csv;
# rostopic echo -p /imu/euler_raw > data_raw.csv;