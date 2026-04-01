function m = link_metrics_snr_prob(lat1, lon1, lat2, lon2, P)

d_km = haversine_km(lat1, lon1, lat2, lon2);
d_m = max(d_km*1000,1);

lambda = P.c / P.fHz;

PL0 = 20*log10(4*pi/lambda);

PL_dB = PL0 + 10*P.nExp*log10(d_m);

A_weather = P.weatherLoss_dB;

Pr_dBm = (P.Pt_dBm + P.Gt_dBi + P.Gr_dBi) - PL_dB - A_weather;

Noise = -174 + 10*log10(P.BHz) + P.NF_dB;

SNR = Pr_dBm - Noise;

Delay = d_m / 3e8;

m.SNR_dB = SNR;
m.Delay_s = Delay;

end