function resample_dataset_clean = preprocess_data(dataset)

% create new table
dataset_clean = table
% extract time 
dataset_clean.t=dataset.t;
% extract lat 
dataset_clean.lat=dataset.lat;
% extract lon 
dataset_clean.lon=dataset.lon;
% extract sog 
dataset_clean.sog=dataset.sog* 0.514444;%knots->m/s
% extract speed_calc  
dataset_clean.speed_calc=dataset.speed_calc
% extract cog  
dataset_clean.cog=deg2rad(dataset.cog);
% extract heading  
dataset_clean.heading=deg2rad(dataset.heading);
%check if datamissing
ismissing(dataset_clean);
% remove missing data
dataset_clean=rmmissing(dataset_clean);
%transfom in timetable
dataset_clean = table2timetable(dataset_clean);
% Determine if the timetable is regular. A regular timetable is one in which the differences between all consecutive row times are the same. outdoors is not a regular timetable.
isregular(dataset_clean);
% Find the differences in the time steps. They vary between half a minute and an hour and a half.
dt = unique(diff(dataset_clean.t));
% resample in second
resample_dataset_clean= retime(dataset_clean,"secondly","spline");
%remove first sample
resample_dataset_clean=resample_dataset_clean(2:end,:)
