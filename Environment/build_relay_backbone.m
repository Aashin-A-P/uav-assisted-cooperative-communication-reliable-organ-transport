function Net = build_relay_backbone(ENV, P)

% ==========================================================
% Build Relay-to-Relay Backbone Network (Option 1 Settings)
% ==========================================================

R = ENV.Relays;
n = height(R);

Adj    = zeros(n,n);
SNR    = nan(n,n);
Psucc  = nan(n,n);
Delay  = nan(n,n);
Dkm    = nan(n,n);

fprintf("\nBuilding relay backbone network...\n");

for i = 1:n
    for j = i+1:n
        
        % Compute link metrics
        m = link_metrics_snr_prob( ...
                R.Latitude(i), R.Longitude(i), ...
                R.Latitude(j), R.Longitude(j), ...
                P, ENV);
        
        % Store values
        Dkm(i,j) = m.d_km;      Dkm(j,i) = m.d_km;
        SNR(i,j) = m.SNR_dB;    SNR(j,i) = m.SNR_dB;
        Psucc(i,j) = m.p_succ;  Psucc(j,i) = m.p_succ;
        Delay(i,j) = m.delay_s; Delay(j,i) = m.delay_s;

        % Edge condition (probability threshold)
        if m.p_succ >= P.pMin
            Adj(i,j) = 1;
            Adj(j,i) = 1;
        end
    end
end

% ==========================================================
% Safe Weighted Graph Construction (Edge List Method)
% ==========================================================

[row, col] = find(triu(Adj,1));  % upper triangle only
weights = zeros(length(row),1);

for k = 1:length(row)
    weights(k) = Delay(row(k), col(k));
end

G = graph(row, col, weights);

% ==========================================================
% Store Outputs
% ==========================================================

Net.Adj        = Adj;
Net.D_km       = Dkm;
Net.SNR_dB     = SNR;
Net.Psucc      = Psucc;
Net.Delay_s    = Delay;
Net.G          = G;

% ==========================================================
% Connectivity Check
% ==========================================================

if numedges(G) == 0
    Net.isConnected = false;
    Net.numComponents = n;
else
    bins = conncomp(G);
    Net.numComponents = numel(unique(bins));
    Net.isConnected   = (Net.numComponents == 1);
end

fprintf("Relay backbone built:\n");
fprintf("  Total Relays      : %d\n", n);
fprintf("  Total Links       : %d\n", numedges(G));
fprintf("  Connected         : %d\n", Net.isConnected);
fprintf("  Components        : %d\n\n", Net.numComponents);

end