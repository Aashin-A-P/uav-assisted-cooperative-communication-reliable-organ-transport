function p = get_packet_priority()

r = rand();

if r < 0.3
    p = "HIGH";
elseif r < 0.7
    p = "MEDIUM";
else
    p = "LOW";
end

end