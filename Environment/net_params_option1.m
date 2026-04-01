function P = net_params_option1()
P.fHz   = 2.4e9;
P.BHz   = 10e6;
P.Pt_dBm = 20;
P.NF_dB  = 7;
P.nExp   = 2.1;
P.Gt_dBi = 10;
P.Gr_dBi = 10;

P.gamma_dB = 10;     % midpoint (0.5 success) if using logistic
P.a = 0.9;           % logistic steepness (tune 0.6..1.5)

P.pMin = 0.05;       % drop links below this probability (graph sparsification)
P.ackOverhead_s = 0.010;  % 10 ms ACK/processing (can tune)
P.payloadBits = 8000;     % 1 KB (adjust for your packet size)
end