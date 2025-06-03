close all
clear
clc

% QC75 datasets loading
% Load files into datasets, renaming columns and extracting DC level
n_files = 83;
folder = 'C:\Mestrado_UA\Dissertacao\Dados_analises\QC75_datasets\';

% Loading
for i = 1:n_files    
    filename = fullfile(folder, sprintf('%d.csv', i));
    raw_data_name = sprintf('data_%d', i);
    eval([raw_data_name ' = readtable(filename, ''Delimiter'', '','');']);    
end

% Renaming column to Force
for i = 1:n_files    
    raw_data_name = sprintf('data_%d', i);
    eval([raw_data_name '.Properties.VariableNames{''filetype5_ActualForce''} = ''Force'';']);
end

% Removing DC level from Force
for i = 1:n_files
    raw_data_name = sprintf('data_%d', i);
    eval([raw_data_name '.Force = ' raw_data_name '.Force - mean(' raw_data_name '.Force);']);
end

% Aggregations
% Sampling parameters
Ts = 0.040; %40 ms
fs = 1/Ts;  %sampling frequency(Hz)
frames = n_files;

% Creating structure to receive vectors of 1 second means
for i = 1:frames
    name = sprintf('df%d', i);
    df.(name) = [];  % Creates df.df1, df.df2, ..., df.df30
end

% Aggregating in 1 second mean samples (EC.IT real case)
samples_sec = 1/Ts;

for i=1:frames
    data_frame = sprintf('data_%d.Force', i);
    data = eval(data_frame);
    n_blocks_sec = floor(length(data)/samples_sec);
    data_temp = data(1:n_blocks_sec*samples_sec);
    reshaped = reshape(data_temp,samples_sec,[]);
    df_temp = sprintf('df%d', i);
    df.(df_temp) = mean(reshaped);
end

% Excluding non operation inputs
k = 1.5;

for d=1:frames
    df_frame = sprintf('df%d', d);
    data = df.(df_frame);
    mad_val = mad(data, 1);
    limiar = k * mad_val;
    df_activity = abs(data) > limiar;
    df_filtered = data(df_activity == true);
    df.(df_frame) = df_filtered;
    %assignin('base', df_frame, df_filtered);
end

% Spectral density
% signalAnalyzer(df.df1)
% signalAnalyzer(df.df2)
% signalAnalyzer(df.df3)
% signalAnalyzer(df.df4)
% signalAnalyzer(df.df5)
% signalAnalyzer(df.df6)
% signalAnalyzer(df.df7)
% signalAnalyzer(df.df8)
% signalAnalyzer(df.df9)
% signalAnalyzer(df.df10)
% signalAnalyzer(df.df11)
% signalAnalyzer(df.df12)
% signalAnalyzer(df.df13)
% signalAnalyzer(df.df14)
% signalAnalyzer(df.df15)

% Features creation
n_features = 9;
features = zeros(frames, n_features);

for idx=1:frames
    df_frame = sprintf('df%d', idx);
    data = df.(df_frame);
    rms_df = rms(data);
    mean_df = mean(data);
    std_df = std(data);
    sk_df = skewness(data);
    k_df = kurtosis(data);
    perc_25 = quantile(data, 0.25);
    perc_50 = quantile(data, 0.50);
    perc_75 = quantile(data, 0.75);
    perc_90 = quantile(data, 0.90);

    features(idx, : ) = [rms_df, mean_df, std_df, sk_df, k_df, perc_25, perc_50, perc_75, perc_90];
end

% Entry for training data preparation
% Creating array of categories
wear = [0 0 1 0 1 1 0 0 0 1 1 0 0 0 0 1 0 1 1 0 0 0 1 1 1 0 0 0 1 1 0 1 1 0 1 0 1 0 0 1 1 0 1 0 0 1 0 0 0 0 0 1 1 1 1 1 0 0 1 0 1 1 1 0 0 0 1 0 0 0 0 1 0 1 0 1 0 0 0 0 0 0 0];
wl = categorical(wear,0:1,{'Eminent','Ok'});

% Normalization
mean_features = mean(features, 1);
sd_features = std(features, [], 1);
features = (features-mean_features) ./ sd_features; %Normalization per feature
features_table = array2table(features);
features_table.Properties.VariableNames = ["rms_df", "mean_df", "std_df", "sk_df", "k_df", "perc_25", "perc_50", "perc_75", "perc_90"];

figure
features_columns = [1, 2, 3, 4, 5, 6, 7, 8, 9];
lables = {'RMS','Mean','Standard Deviation','Skewness','Kurtosis','Percent 25','Percent 50','Percent 75','Percent 90'}
for i = 1:length(features_columns)
    subplot(3,3,i)  
    histogram(features(:, features_columns(i)), 'FaceColor', [0.2+i*0.01, 0.4, 0.6])
    title(lables{i})
    xlabel('Value')
    ylabel('Frequency')
    grid on
end



% Testing Model
% QC74
% Load file into data, renames columns and extracts DC level
n_files = 15;
folder = 'C:\Mestrado_UA\Dissertacao\Dados_analises\QC74_samples\';

% Loading
for i = 1:n_files    
    filename = fullfile(folder, sprintf('%d.csv', i));
    raw_data_name = sprintf('data_test_%d', i);
    eval([raw_data_name ' = readtable(filename, ''Delimiter'', '','');']);
end

% Renaming column to Force
for i = 1:n_files    
    raw_data_name = sprintf('data_test_%d', i);
    eval([raw_data_name '.Properties.VariableNames{''filetype5_ActualForce''} = ''Force'';']);
end

% Removing DC level from Force
for i = 1:n_files
    raw_data_name = sprintf('data_test_%d', i);
    eval([raw_data_name '.Force = ' raw_data_name '.Force - mean(' raw_data_name '.Force);']);
end


% Sampling parameters
Ts = 0.040; %40 ms
fs = 1/Ts;  %sampling frequency(Hz)
frames = n_files;

% Creating structure to receive vectors of 1 second means
for i = 1:frames
    name = sprintf('df%d', i);
    df_test.(name) = [];  % Creates df.df1, df.df2, ..., df.df15
end

% Aggregating in 1 second mean samples (EC.IT real case)
samples_sec = 1/Ts;

for i=1:frames
    data_frame = sprintf('data_test_%d.Force', i);
    data = eval(data_frame);
    n_blocks_sec = floor(length(data)/samples_sec);
    data_temp = data(1:n_blocks_sec*samples_sec);
    reshaped = reshape(data_temp,samples_sec,[]);
    df_temp = sprintf('df%d', i);
    df_test.(df_temp) = mean(reshaped);
end

% Excluding non operation inputs
k = 1.5;

for d=1:frames
    df_frame = sprintf('df%d', d);
    data = df_test.(df_frame);
    mad_val = mad(data, 1);
    limiar = k * mad_val;
    df_activity = abs(data) > limiar;
    df_filtered = data(df_activity == true);
    df_test.(df_frame) = df_filtered;
    %assignin('base', df_frame, df_filtered);
end

% Features extraction
n_features = 9;
features_test = zeros(frames, n_features);

for idx=1:frames
    df_frame = sprintf('df%d', idx);
    data = df_test.(df_frame);
    rms_df = rms(data);
    mean_df = mean(data);
    std_df = std(data);
    sk_df = skewness(data);
    k_df = kurtosis(data);
    perc_25 = quantile(data, 0.25);
    perc_50 = quantile(data, 0.50);
    perc_75 = quantile(data, 0.75);
    perc_90 = quantile(data, 0.90);

    features_test(idx, : ) = [rms_df, mean_df, std_df, sk_df, k_df, perc_25, perc_50, perc_75, perc_90];
end

features_test = (features_test-mean_features) ./ sd_features; % Normalization -> Test data set is normalized with the mean and std used in training
features_test_table = array2table(features_test);
features_test_table.Properties.VariableNames = ["rms_df", "mean_df", "std_df", "sk_df", "k_df", "perc_25", "perc_50", "perc_75", "perc_90"];

[yfit,scores] = trainedModel.predictFcn(features_test_table(1:15,:));
disp(yfit);
figure;
plot(1:15, yfit, 'o-', 'LineWidth', 2);
xlabel('Sample Index');
ylabel('Predicted Value');
title('Predictions for First 15 Samples');
grid on;