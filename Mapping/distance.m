clc; clear; close all;

%% ================== HOSPITAL DATA (ALL 39) ==================
% {Name, Latitude, Longitude}

H = {
    "MIOT Hospitals", 13.021221305087122, 80.18399507329879
    "Kumaran Hospitals", 13.079188595485835, 80.24908781090282
    "Sri Ramachandra Medical Centre", 13.03925226152519, 80.14349035322947
    "Gleneagles Health City", 12.89805795018799, 80.20624846857314
    "Dr Mehta Multispecialty Hospital", 13.07186170709951, 80.24065806857554
    "Dr Kamakshi Memorial Hospitals", 12.95184673893021, 80.2094058632643
    "Government Royapettah Hospital", 13.055498069165232, 80.2648511955569
    "Rajiv Gandhi Govt General Hospital", 13.08153114180747, 80.2776430685756
    "Govt Kilpauk Medical College Hospital", 13.077541441916278, 80.2419763867904
    "Apollo Hospitals Greams Road", 13.078772657560947, 80.24588263126917
    "Billroth Hospital R A Puram", 13.027678951973726, 80.25664572439291
    "Govt Stanley Medical Hospital", 13.106229426480258, 80.28589728021201
    "The Madras Medical Mission", 13.08609508989842, 80.18714099555731
    "Fortis Malar Hospital", 13.010384794385843, 80.25857588664216
    "Bharathi Rajaa Hospital", 13.048118402400004, 80.24497885480359
    "Chettinad Hospital & Research Institute", 12.796856116913377, 80.218488188645
    "Frontline Hospital & Research Institute", 13.1022933105255, 80.19031625570929
    "Sree Balaji Medical College & Hospital", 12.95541658997134, 80.13781040081767
    "Kauvery Hospital", 13.039492266036037, 80.25730776380318
    "SRM Medical College (Kattankulathur)", 12.82104353039475, 80.04814100904443
    "SIMS Hospital", 13.053273688502998, 80.21101102786939
    "Apollo Speciality Ayanambakkam", 13.071034364020369, 80.1506202955571
    "Velammal Medical College Hospital", 13.079292822735503, 80.11440874156483
    "Saveetha Medical College & Hospital", 13.026403198082061, 80.01397525156452
    "Prashanth Superspeciality Hospital", 12.978696263453147, 80.2213826397378
    "MGM Healthcare", 13.071237358593248, 80.22175142624829
    "GEM Hospital", 12.96941396988622, 80.2458926721459
    "Hindu Mission Hospital", 12.923851514454665, 80.11410716671865
    "AINU Nephrology & Urology", 13.056415062415994, 80.24781706857527
    "Bhaarath Medical College & Hospital", 12.916904962984944, 80.14187527480817
    "Chennai Urology & Robotics Institute", 12.957871649407371, 80.24427710406547
    "Kauvery Hospital Kovilambakkam", 12.948425988128676, 80.18972468206456
    "Kauvery Hospital Vadapalani", 13.047776450177288, 80.20422187439316
    "Karpagam Hospital", 13.200639912595816, 79.89083476390348
    "Vinita Hospital", 13.054697372992095, 80.2454149109025
    "SRM Global Hospitals", 12.826358287765219, 80.04139912253528
    "Tagore Medical College & Hospital", 12.860384521955428, 80.13614546857262
    "Rainbow Children’s Hospital", 13.01427034497125, 80.22254416486514
    "HY Care Hospitals", 13.071071771801899, 80.21533425676145
};

N = size(H,1);
names = string(H(:,1));
lat = cell2mat(H(:,2));
lon = cell2mat(H(:,3));

%% ================== AIRPORT ==================
airportName = "Chennai International Airport";
airportLatLon = [12.99659175499222, 80.1708075900248];

%% ================== LAT/LON → METERS ==================
R = 6371000;
lat0 = mean(lat) * pi/180;
lon0 = mean(lon) * pi/180;

x = R * ((lon*pi/180) - lon0) .* cos(lat0);
y = R * ((lat*pi/180) - lat0);
Hospitals = [x y];

ax = R * ((airportLatLon(2)*pi/180) - lon0) .* cos(lat0);
ay = R * ((airportLatLon(1)*pi/180) - lat0);
airportPos = [ax ay];

%% ================== VISUALIZATION ==================
figure('Color','w','Position',[80 80 1300 720]);
hold on; axis equal; grid on;

title("Chennai Hospital Network (39 Hospitals + Airport)");
xlabel("X (meters)");
ylabel("Y (meters)");

plot(Hospitals(:,1),Hospitals(:,2),'ks','MarkerSize',8,'MarkerFaceColor','y');
plot(airportPos(1),airportPos(2),'kp','MarkerSize',14,'MarkerFaceColor',[0.2 0.6 1]);

for i = 1:N
    text(Hospitals(i,1)+250, Hospitals(i,2)+250, ...
        sprintf("H%d",i),'FontWeight','bold','FontSize',9);
end

text(airportPos(1)+300, airportPos(2)+300, ...
    "Airport",'FontWeight','bold','Color',[0 0.3 0.6]);

legend("Hospitals","Airport","Location","northoutside");

%% ================== DISTANCE MATRIX ==================
disp(" ");
disp("===== PAIRWISE DISTANCE MATRIX (km) =====");

D = zeros(N,N);
for i = 1:N
    for j = 1:N
        D(i,j) = norm(Hospitals(i,:) - Hospitals(j,:)) / 1000;
    end
end

fprintf("%6s"," ");
for j = 1:N
    fprintf("%6s",sprintf("H%d",j));
end
fprintf("\n");

for i = 1:N
    fprintf("%6s",sprintf("H%d",i));
    for j = 1:N
        fprintf("%6.1f",D(i,j));
    end
    fprintf("\n");
end

%% ================== NEAREST HOSPITAL ANALYSIS ==================
disp(" ");
disp("===== NEAREST HOSPITAL FOR EACH NODE =====");
for i = 1:N
    d = D(i,:);
    d(i) = inf;
    [m,idx] = min(d);
    fprintf("H%-2d (%s) → Nearest: H%-2d | %.2f km\n", ...
        i, names(i), idx, m);
end

%% ================== LEGEND PRINT ==================
disp(" ");
disp("===== HOSPITAL LEGEND =====");
for i = 1:N
    fprintf("H%-2d : %s\n", i, names(i));
end
disp("A  : Chennai International Airport");
