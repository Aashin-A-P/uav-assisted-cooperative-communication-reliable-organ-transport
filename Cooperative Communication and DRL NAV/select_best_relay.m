function best = select_best_relay(T)

SNRn = normalize(T.SNR_dB);
Dn   = normalize(T.Delay_s);

score = 0.7*SNRn - 0.3*Dn;

[~,i] = max(score);

best = T(i,:);

end