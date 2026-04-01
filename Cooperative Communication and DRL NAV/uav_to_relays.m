function T = uav_to_relays(ENV, P, lat, lon)

R = ENV.Relays;
n = height(R);

SNR = zeros(n,1);
Delay = zeros(n,1);

for k = 1:n

    m = link_metrics_snr_prob(lat, lon,...
        R.Latitude(k), R.Longitude(k), P);

    SNR(k) = m.SNR_dB;
    Delay(k) = m.Delay_s;
end

T = table((1:n)', SNR, Delay,...
    'VariableNames',{'idx','SNR_dB','Delay_s'});

end