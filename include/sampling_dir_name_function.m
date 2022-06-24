function sampling_dir_name = sampling_dir_name_function(lidarConfig)
    sampling_dir_name = ['sampling_' num2str(lidarConfig.num_lasers) '_' num2str(lidarConfig.img_length) '_' num2str(lidarConfig.length_percentage) '_' num2str(lidarConfig.min_length) '_' num2str(lidarConfig.row_times) '_' num2str(lidarConfig.col_times)];
end