function coefficients_dir_name = coefficients_dir_name_function(thresholdConfig,patchConfig)
    coefficients_dir_name = ['coefficients_' num2str(thresholdConfig.avg_filter_thres) '_' num2str(thresholdConfig.rmv_filter_thres) '_' num2str(thresholdConfig.avg_error_thres_patch) '_' num2str(thresholdConfig.remove) '_' num2str(patchConfig.patch_row_rso) '_' num2str(patchConfig.patch_col_rso) '_' num2str(patchConfig.row_overlap_step) '_' num2str(patchConfig.col_overlap_step) '_' num2str(patchConfig.k)];
end

