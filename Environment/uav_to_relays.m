function U = uav_to_relays(ENV, P, uavLat, uavLon)

R = ENV.Relays;
n = height(R);

U.SNR_dB = nan(n,1);
U.Psucc  = nan(n,1);
U.Delay_s = nan(n,1);
U.D_km   = nan(n,1);

for k = 1:n
    m = link_metrics_snr_prob(uavLat, uavLon, R.Latitude(k), R.Longitude(k), P, ENV);
    U.D_km(k) = m.d_km;
    U.SNR_dB(k) = m.SNR_dB;
    U.Psucc(k) = m.p_succ;
    U.Delay_s(k) = m.delay_s;
end
end