clc; clear;

[names,~,~,~,~,HospitalsXY] = hospital_data();

src = 1;
dst = 9;

relay = (HospitalsXY(src,:) + HospitalsXY(dst,:)) / 2;

d1 = norm(relay - HospitalsXY(src,:));
d2 = norm(HospitalsXY(dst,:) - relay);

Pt = 20;
noise = -100;
pathLossExp = 2.5;

SINR1 = Pt - (10*pathLossExp*log10(d1)) - noise;
SINR2 = Pt - (10*pathLossExp*log10(d2)) - noise;

SINR_eff = min(SINR1, SINR2);

fprintf("SINGLE RELAY BASELINE\n");
fprintf("Source → Relay → Destination\n");
fprintf("Effective SINR: %.2f dB\n", SINR_eff);