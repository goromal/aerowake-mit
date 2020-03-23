bagname = '../bags/312_flight_bags/3-fell_why/2020-03-12-FELLWHY.bag';
addpath(genpath('matlab_utilities/'));
bagdata = processAllROSBagTopics(bagname, false);

start_time = 22;
end_time = 30;

t_or = bagdata.output_raw.t;
or_idx0 = idx_of_val(t_or, start_time);
or_idxf = idx_of_val(t_or, end_time);
t_or = t_or(or_idx0:or_idxf);
s_or = double(bagdata.output_raw.values(1,or_idx0:or_idxf));

t_rr = bagdata.rc_raw.t;
rr_idx0 = idx_of_val(t_rr, start_time);
rr_idxf = idx_of_val(t_rr, end_time);
t_rr = t_rr(rr_idx0:rr_idxf);
c1_rr = double(bagdata.rc_raw.vals(1,rr_idx0:rr_idxf)) / 1000.0;
c2_rr = double(bagdata.rc_raw.vals(2,rr_idx0:rr_idxf)) / 1000.0;
c3_rr = double(bagdata.rc_raw.vals(3,rr_idx0:rr_idxf)) / 1000.0;
c4_rr = double(bagdata.rc_raw.vals(4,rr_idx0:rr_idxf)) / 1000.0;
c5_rr = double(bagdata.rc_raw.vals(5,rr_idx0:rr_idxf)) / 1000.0;
c6_rr = double(bagdata.rc_raw.vals(6,rr_idx0:rr_idxf)) / 1000.0;

plot(1:1:length(s_or), s_or, 'ko', ...
     1:1:length(c1_rr), c1_rr, 'k+', ...
     1:1:length(c2_rr), c2_rr, 'r+', ...
     1:1:length(c3_rr), c3_rr, 'g+', ...
     1:1:length(c4_rr), c4_rr, 'b+', ...
     1:1:length(c5_rr), c5_rr, 'c*', ...
     1:1:length(c6_rr), c6_rr, 'y*')
legend('Motor Output','A','E','T','R','O','ARM')
grid on