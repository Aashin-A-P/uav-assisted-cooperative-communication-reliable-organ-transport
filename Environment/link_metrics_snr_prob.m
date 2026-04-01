function M = link_metrics_snr_prob(lat1, lon1, lat2, lon2, P, ENV)

% Distance
d_km = haversine_km(lat1, lon1, lat2, lon2);
d_m = max(d_km*1000, 1);

% Noise floor (dBm)
N_dBm = -174 + 10*log10(P.BHz) + P.NF_dB;

% Path loss (log-distance, anchored at 1m with FSPL)
c = 3e8;
lambda = c / P.fHz;
PL0 = 20*log10(4*pi*1/lambda);           % FSPL at 1 m
PL_dB = PL0 + 10*P.nExp*log10(d_m/1);

% Weather penalty (optional, very light — keep small)
% Use midpoint weather between nodes
midLat = (lat1 + lat2)/2; midLon = (lon1 + lon2)/2;
W = getWeatherAt(ENV, midLat, midLon);
A_weather_dB = 0.02*(W.HumidityPct) + 0.10*(W.WindSpeedMS);  % tune coefficients

% Received power & SNR
Pr_dBm = (P.Pt_dBm + P.Gt_dBi + P.Gr_dBi) - PL_dB - A_weather_dB;
SNR_dB = Pr_dBm - N_dBm;

% Success probability
p_succ = snr_to_psucc(SNR_dB, P.gamma_dB, P.a);

% Expected transmissions
Etx = 1 / p_succ;

% Tx time (payload / bandwidth) (simple)
txTime_s = P.payloadBits / P.BHz;

% Expected link delay
delay_s = Etx * (txTime_s + P.ackOverhead_s);

M.d_km = d_km;
M.SNR_dB = SNR_dB;
M.p_succ = p_succ;
M.Etx = Etx;
M.delay_s = delay_s;
end