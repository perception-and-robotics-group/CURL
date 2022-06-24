function [save_sampling_dir,sampling_remove_dir,save_coefficients_dir,save_recons_dir,save_evaluation_error_dis_file,save_evaluation_compression_file] = create_dir_function(saving_dir,sequence,frame_num,lidarConfig,thresholdConfig,patchConfig,reconsConfig)
    sampling_dir_name = sampling_dir_name_function(lidarConfig);
    save_sampling_dir= [saving_dir '/' num2str(sequence) '/' num2str(frame_num-1) '/' sampling_dir_name];
    sampling_remove_dir_name = sampling_remove_dir_name_function(thresholdConfig);
    sampling_remove_dir = [save_sampling_dir '/' sampling_remove_dir_name];
    coefficients_dir_name = coefficients_dir_name_function(thresholdConfig,patchConfig);
    save_coefficients_dir = [sampling_remove_dir '/' coefficients_dir_name];
    save_recons_dir_name = save_recons_dir_name_function(reconsConfig);
    save_recons_dir = [save_coefficients_dir '/' save_recons_dir_name];
    save_evaluation_error_dis_file = [save_recons_dir '/error_dis.mat'];
    save_evaluation_compression_file = [save_coefficients_dir '/compression_rate.mat'];
end

