clear all;
close all;


file_dir_all = {
    '.\Data\YH100';...
    };

for i_file = 1:length(file_dir_all)
    
    func_export_autocage_data_to_matfile(file_dir_all{i_file});
    
end


