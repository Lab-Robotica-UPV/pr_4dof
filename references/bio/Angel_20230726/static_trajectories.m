clear
clc
close all

files = ["TrayCarte_mus1_Angel", "TrayCarte_mus7_Angel", "TrayCarte_mus14_Angel", "TrayPatas_mus1_Angel", "TrayPatas_mus7_Angel", "TrayPatas_mus14_Angel"];

for i=1:numel(files)
    file = load(strcat(files(i), ".txt"));
    sample = file(1,:);
    tray = ones(12000,4) .* sample;
    name_file_final = strcat(files(i),"_Estatica.txt");
    writematrix(tray, name_file_final, 'Delimiter', ' ');
end